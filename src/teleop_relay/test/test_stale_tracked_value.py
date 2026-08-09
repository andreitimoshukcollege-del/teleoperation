from teleop_relay.stale_tracked_value import StaleTrackedValue


def test_never_updated_reads_as_invalid():
    value = StaleTrackedValue(staleness_timeout_seconds=0.2)

    valid, degrees = value.read(now=100.0)

    assert valid is False
    assert degrees == 0


def test_fresh_update_reads_as_valid():
    value = StaleTrackedValue(staleness_timeout_seconds=0.2)

    value.update(42, now=100.0)
    valid, degrees = value.read(now=100.05)

    assert valid is True
    assert degrees == 42


def test_update_older_than_timeout_reads_as_invalid():
    value = StaleTrackedValue(staleness_timeout_seconds=0.2)

    value.update(42, now=100.0)
    valid, degrees = value.read(now=100.3)

    assert valid is False


def test_update_exactly_at_timeout_boundary_still_reads_as_valid():
    value = StaleTrackedValue(staleness_timeout_seconds=0.2)

    value.update(42, now=100.0)
    valid, _ = value.read(now=100.2)

    assert valid is True


def test_second_update_before_timeout_keeps_it_valid_and_returns_latest_value():
    value = StaleTrackedValue(staleness_timeout_seconds=0.2)

    value.update(42, now=100.0)
    value.update(43, now=100.1)
    valid, degrees = value.read(now=100.25)

    assert valid is True
    assert degrees == 43


def test_a_channel_that_stops_updating_eventually_reads_as_invalid_again():
    # Regression test for the real bug this class fixes: the middle-arm servo's reads never
    # succeed (robot_controller_node.py's own comment), so a single early/lucky reading must
    # not be reported as valid forever.
    value = StaleTrackedValue(staleness_timeout_seconds=0.2)

    value.update(60, now=100.0)
    assert value.read(now=100.05)[0] is True

    valid, degrees = value.read(now=500.0)

    assert valid is False
