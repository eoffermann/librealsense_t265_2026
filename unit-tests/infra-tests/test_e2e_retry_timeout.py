# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

"""
E2E: pytest-timeout resets between pytest-retry attempts.

pytest-timeout arms its per-item timer in `pytest_runtest_protocol`, whose
yield covers setup + all call attempts + teardown. pytest-retry re-invokes
`pytest_runtest_call` from `pytest_runtest_makereport`, still under that
outer yield — so without a reset, retries share the original --timeout
budget. A test that consumes most of the budget on attempt 1 gets killed
mid-attempt-2 with a `+++ Timeout +++` stack dump. The conftest
`_reset_pytest_timeout_for_retry` re-arms the timer at the start of every
`pytest_runtest_call`. This test locks in the fixed behaviour.
"""

from helpers import run_e2e, parse_outcomes


class TestRetryTimeoutReset:

    def test_timeout_resets_between_retry_attempts(self):
        """Attempt 1 sleeps 3s and fails; attempt 2 sleeps 3s and passes.
        With --timeout=5, the shared-timer bug would kill attempt 2 at ~2s.
        With the reset, each attempt gets a fresh 5s and the test PASSES.
        2s slack per attempt tolerates slow CI scheduler jitter."""
        rc, out, *_ = run_e2e("pytest-retry-timeout.py",
                               "--retries", "1", "--timeout", "5")
        assert rc == 0, out
        outcomes = parse_outcomes(out)
        assert outcomes.get("passed") == 1, out
        assert outcomes.get("retried") == 1, f"expected exactly 1 retry: {out}"
        assert "+++ Timeout +++" not in out, out
