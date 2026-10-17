"""Tests for the ROS-free diagnostics maths."""

import pytest

from dmu_vision_tools.topic_probe import (
    Durability,
    QosProfile,
    RateEstimator,
    Reliability,
    Report,
    StalenessTracker,
    Status,
    effective_durability,
    effective_reliability,
    format_age,
    format_rate,
    qos_compatible,
)


# --------------------------------------------------------------------------
# QoS interoperability: the check that turns a silent dead topic into a verdict
# --------------------------------------------------------------------------
def test_reliable_publisher_satisfies_both_best_effort_and_reliable_subscribers():
    pub = QosProfile(reliability=Reliability.RELIABLE)
    assert qos_compatible(pub, QosProfile(reliability=Reliability.BEST_EFFORT)).compatible
    assert qos_compatible(pub, QosProfile(reliability=Reliability.RELIABLE)).compatible


def test_best_effort_publisher_cannot_satisfy_a_reliable_subscriber():
    """This is the real failure we hit with image_proc (Phase 2)."""
    pub = QosProfile(reliability=Reliability.BEST_EFFORT, node_name="/camera_node")
    sub = QosProfile(reliability=Reliability.RELIABLE, node_name="/camera/RectifyNode")
    verdict = qos_compatible(pub, sub)
    assert not verdict.compatible
    assert "no data will ever be delivered" in verdict.reason
    assert "/camera_node" in verdict.reason and "/camera/RectifyNode" in verdict.reason


def test_best_effort_publisher_satisfies_a_best_effort_subscriber():
    """This is the yolo_ros case with image_reliability: 2."""
    pub = QosProfile(reliability=Reliability.BEST_EFFORT)
    sub = QosProfile(reliability=Reliability.BEST_EFFORT)
    assert qos_compatible(pub, sub).compatible


def test_volatile_publisher_cannot_satisfy_a_transient_local_subscriber():
    pub = QosProfile(durability=Durability.VOLATILE)
    sub = QosProfile(durability=Durability.TRANSIENT_LOCAL)
    verdict = qos_compatible(pub, sub)
    assert not verdict.compatible
    assert "TRANSIENT_LOCAL" in verdict.reason


def test_transient_local_publisher_satisfies_a_volatile_subscriber():
    pub = QosProfile(durability=Durability.TRANSIENT_LOCAL)
    sub = QosProfile(durability=Durability.VOLATILE)
    assert qos_compatible(pub, sub).compatible


def test_system_default_resolves_like_ros2_does():
    assert effective_reliability(Reliability.SYSTEM_DEFAULT) is Reliability.RELIABLE
    assert effective_durability(Durability.SYSTEM_DEFAULT) is Durability.VOLATILE
    # default publisher is effectively RELIABLE -> satisfies a BEST_EFFORT subscriber
    assert qos_compatible(QosProfile(), QosProfile(reliability=Reliability.BEST_EFFORT)).compatible


@pytest.mark.parametrize(
    "pub_rel,sub_rel,expected",
    [
        (Reliability.RELIABLE, Reliability.RELIABLE, True),
        (Reliability.RELIABLE, Reliability.BEST_EFFORT, True),
        (Reliability.BEST_EFFORT, Reliability.BEST_EFFORT, True),
        (Reliability.BEST_EFFORT, Reliability.RELIABLE, False),
        (Reliability.RELIABLE, Reliability.SYSTEM_DEFAULT, True),
        (Reliability.BEST_EFFORT, Reliability.SYSTEM_DEFAULT, False),
    ],
)
def test_reliability_matrix(pub_rel, sub_rel, expected):
    verdict = qos_compatible(QosProfile(reliability=pub_rel), QosProfile(reliability=sub_rel))
    assert verdict.compatible is expected


def test_verdict_description_names_both_endpoints():
    pub = QosProfile(reliability=Reliability.BEST_EFFORT, node_name="/a")
    text = pub.describe()
    assert "best_effort" in text and "/a" in text


# --------------------------------------------------------------------------
# Rate / staleness
# --------------------------------------------------------------------------
def test_rate_needs_two_samples():
    est = RateEstimator()
    assert est.rate_hz() is None
    est.add(100.0)
    assert est.rate_hz() is None
    est.add(100.1)
    assert est.rate_hz() == pytest.approx(10.0)


def test_rate_uses_sliding_window():
    est = RateEstimator(window=3)
    for t in (0.0, 0.1, 0.2, 0.3, 0.4):
        est.add(t)
    assert est.count == 3
    assert est.rate_hz() == pytest.approx(10.0)


def test_rate_guards_against_zero_span():
    est = RateEstimator()
    est.add(5.0)
    est.add(5.0)
    assert est.rate_hz() is None


def test_rate_rejects_invalid_window():
    with pytest.raises(ValueError):
        RateEstimator(window=1)


def test_staleness_before_and_after_a_message():
    tracker = StalenessTracker()
    assert tracker.age_s(100.0) is None
    assert tracker.count == 0
    tracker.add(100.0)
    assert tracker.count == 1
    assert tracker.age_s(100.25) == pytest.approx(0.25)


# --------------------------------------------------------------------------
# Report / verdict
# --------------------------------------------------------------------------
def test_report_exit_code_is_zero_with_only_warnings():
    report = Report()
    report.add("a", Status.OK, "fine")
    report.add("b", Status.WARN, "meh")
    assert report.ok
    assert report.exit_code == 0
    assert "PASS WITH WARNINGS" in report.render()


def test_report_exit_code_is_one_with_a_failure():
    report = Report()
    report.add("a", Status.OK, "fine")
    report.add("b", Status.FAIL, "broken")
    assert not report.ok
    assert report.exit_code == 1
    assert "RESULT: FAIL" in report.render()
    assert len(report.failures) == 1


def test_report_all_ok():
    report = Report()
    report.add("a", Status.OK, "fine")
    assert report.exit_code == 0
    assert "RESULT: PASS" in report.render()


def test_report_line_format_is_stable():
    report = Report()
    report.add("topic image", Status.OK, "/camera/image_raw has data")
    line = report.results[0].line()
    assert line.startswith("[OK  ] topic image")


def test_format_helpers():
    assert format_rate(None) == "n/a"
    assert format_rate(9.876) == "9.88 Hz"
    assert format_age(None) == "n/a"
    assert format_age(0.1234) == "0.123 s"
