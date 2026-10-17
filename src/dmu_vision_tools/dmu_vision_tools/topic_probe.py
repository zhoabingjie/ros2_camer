"""Pure, ROS-free diagnostics maths.

Everything in this module is deterministic and clock-injected so it can be unit
tested without a ROS graph. The ROS-facing layer (health_check.py) only adapts
introspection data into these types.

The headline check is :func:`qos_compatible`. Two of this project's real bugs were
QoS mismatches whose symptom was "the topic exists, the node is running, and no
data ever arrives, with no error anywhere":

  * image_proc's rectify_node subscribing RELIABLE to our BEST_EFFORT camera (Phase 2)
  * yolo_ros' image_reliability parameter, where 1 means RELIABLE and our camera
    publishes BEST_EFFORT (Phase 3)

A pairwise publisher/subscriber compatibility check turns that class of failure
into a one-line verdict.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Deque, List, Optional, Sequence, Tuple


class Reliability(str, Enum):
    SYSTEM_DEFAULT = "system_default"
    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"


class Durability(str, Enum):
    SYSTEM_DEFAULT = "system_default"
    TRANSIENT_LOCAL = "transient_local"
    VOLATILE = "volatile"


def effective_reliability(value: Reliability) -> Reliability:
    """Resolve SYSTEM_DEFAULT the way ROS 2 does for topic QoS."""
    return Reliability.RELIABLE if value is Reliability.SYSTEM_DEFAULT else value


def effective_durability(value: Durability) -> Durability:
    return Durability.VOLATILE if value is Durability.SYSTEM_DEFAULT else value


@dataclass(frozen=True)
class QosProfile:
    """The subset of a QoS profile that decides interoperability."""

    reliability: Reliability = Reliability.RELIABLE
    durability: Durability = Durability.VOLATILE
    depth: int = 10
    node_name: str = ""

    def describe(self) -> str:
        return (
            f"{self.node_name or '?'}: "
            f"reliability={effective_reliability(self.reliability).value} "
            f"durability={effective_durability(self.durability).value} "
            f"depth={self.depth}"
        )


@dataclass(frozen=True)
class QosVerdict:
    """Result of one publisher/subscriber compatibility check."""

    compatible: bool
    reason: str
    publisher: QosProfile
    subscriber: QosProfile


def endpoint_pair(publisher: QosProfile, subscriber: QosProfile) -> str:
    """Prefix naming both endpoints, when the graph told us who they are.

    A verdict that says which nodes disagree is the difference between a usable
    diagnostic and a scavenger hunt, so every message carries it when available.
    """
    if publisher.node_name and subscriber.node_name:
        return f"{publisher.node_name} -> {subscriber.node_name}: "
    return ""


def qos_compatible(publisher: QosProfile, subscriber: QosProfile) -> QosVerdict:
    """Apply the DDS/ROS 2 request-offer rules that decide if data will flow.

    A publisher *offers*; a subscriber *requests*. The publisher must offer at
    least what the subscriber requests:

      * RELIABLE offer satisfies BEST_EFFORT and RELIABLE requests.
        BEST_EFFORT offer satisfies only BEST_EFFORT requests.
      * TRANSIENT_LOCAL offer satisfies VOLATILE and TRANSIENT_LOCAL requests.
        VOLATILE offer satisfies only VOLATILE requests.
    """
    pub_rel = effective_reliability(publisher.reliability)
    sub_rel = effective_reliability(subscriber.reliability)
    pub_dur = effective_durability(publisher.durability)
    sub_dur = effective_durability(subscriber.durability)
    pair = endpoint_pair(publisher, subscriber)

    if pub_rel is Reliability.BEST_EFFORT and sub_rel is Reliability.RELIABLE:
        return QosVerdict(
            False,
            pair + "publisher offers BEST_EFFORT but subscriber requests RELIABLE "
            "-> no data will ever be delivered (fix the subscriber's reliability, or "
            "make the publisher RELIABLE)",
            publisher,
            subscriber,
        )
    if pub_dur is Durability.VOLATILE and sub_dur is Durability.TRANSIENT_LOCAL:
        return QosVerdict(
            False,
            pair + "publisher offers VOLATILE but subscriber requests TRANSIENT_LOCAL "
            "-> no data will ever be delivered",
            publisher,
            subscriber,
        )
    return QosVerdict(
        True,
        pair + f"offers ({pub_rel.value}/{pub_dur.value}) >= "
        f"requests ({sub_rel.value}/{sub_dur.value})",
        publisher,
        subscriber,
    )


@dataclass
class RateEstimator:
    """Windowed rate estimator over arrival timestamps."""

    window: int = 20
    _samples: Deque[float] = field(default_factory=deque)

    def __post_init__(self) -> None:
        if self.window < 2:
            raise ValueError("window must be >= 2")
        self._samples = deque(maxlen=self.window)

    def add(self, now: float) -> None:
        self._samples.append(float(now))

    @property
    def count(self) -> int:
        return len(self._samples)

    def rate_hz(self) -> Optional[float]:
        """None until at least two samples span a positive interval."""
        if len(self._samples) < 2:
            return None
        span = self._samples[-1] - self._samples[0]
        if span <= 0:
            return None
        return (len(self._samples) - 1) / span


@dataclass
class StalenessTracker:
    """Tracks how long ago the most recent message arrived, and whether we got any."""

    _last: Optional[float] = None
    _count: int = 0

    def add(self, now: float) -> None:
        self._last = float(now)
        self._count += 1

    @property
    def count(self) -> int:
        return self._count

    def age_s(self, now: float) -> Optional[float]:
        if self._last is None:
            return None
        return float(now) - self._last


class Status(str, Enum):
    OK = "ok"
    WARN = "warn"
    FAIL = "fail"


@dataclass
class CheckResult:
    name: str
    status: Status
    detail: str

    def line(self) -> str:
        return f"[{self.status.value.upper():4}] {self.name:<34} {self.detail}"


@dataclass
class Report:
    results: List[CheckResult] = field(default_factory=list)

    def add(self, name: str, status: Status, detail: str) -> CheckResult:
        result = CheckResult(name, status, detail)
        self.results.append(result)
        return result

    @property
    def failures(self) -> List[CheckResult]:
        return [r for r in self.results if r.status is Status.FAIL]

    @property
    def warnings(self) -> List[CheckResult]:
        return [r for r in self.results if r.status is Status.WARN]

    @property
    def ok(self) -> bool:
        return not self.failures

    @property
    def exit_code(self) -> int:
        return 1 if self.failures else 0

    def render(self) -> str:
        lines = [r.line() for r in self.results]
        lines.append("-" * 78)
        if self.failures:
            lines.append(f"RESULT: FAIL ({len(self.failures)} failure(s), {len(self.warnings)} warning(s))")
        elif self.warnings:
            lines.append(f"RESULT: PASS WITH WARNINGS ({len(self.warnings)} warning(s))")
        else:
            lines.append("RESULT: PASS")
        return "\n".join(lines)


def format_rate(rate: Optional[float]) -> str:
    return "n/a" if rate is None else f"{rate:.2f} Hz"


def format_age(age: Optional[float]) -> str:
    return "n/a" if age is None else f"{age:.3f} s"
