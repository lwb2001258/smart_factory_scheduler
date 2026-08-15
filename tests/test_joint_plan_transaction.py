import sys
import unittest
from pathlib import Path

SUPERVISOR_DIR = (Path(__file__).resolve().parents[1] / "controllers" /
                  "factory_supervisor")
sys.path.insert(0, str(SUPERVISOR_DIR))

from joint_plan_transaction import JointPlanTransaction


class JointPlanTransactionTests(unittest.TestCase):
    def transaction(self):
        return JointPlanTransaction(
            epoch=7, plans={1: [(0, 0)], 2: [(1, 0)]},
            versions={1: 3, 2: 4}, created_at=10.0, deadline=10.5)

    def test_all_members_must_ack_before_activation(self):
        txn = self.transaction()
        self.assertTrue(txn.acknowledge(1, True))
        self.assertIsNone(txn.decision(10.1))
        self.assertTrue(txn.acknowledge(2, True))
        self.assertEqual("ready", txn.decision(10.2))
        txn.begin_arming(10.7)
        txn.acknowledge_armed(1)
        txn.acknowledge_armed(2)
        txn.begin_commit(11.2)
        txn.acknowledge_committed(1)
        txn.acknowledge_committed(2)
        txn.acknowledge_activated(1)
        txn.acknowledge_activated(2)
        txn.mark_activated()
        self.assertEqual("activated", txn.state)

    def test_one_rejection_aborts_entire_group(self):
        txn = self.transaction()
        txn.acknowledge(1, True)
        txn.acknowledge(2, False)
        self.assertEqual("aborted", txn.decision(10.2))

    def test_missing_ack_times_out_without_partial_activation(self):
        txn = self.transaction()
        txn.acknowledge(1, True)
        self.assertEqual("aborted", txn.decision(10.5))
        self.assertEqual("prepare_timeout", txn.failure_reason)

    def test_stale_or_unknown_ack_is_ignored(self):
        txn = self.transaction()
        self.assertFalse(txn.acknowledge(99, True))
        txn.acknowledge(1, True)
        txn.acknowledge(2, True)
        txn.decision(10.1)
        txn.begin_arming(10.6)
        txn.acknowledge_armed(1)
        txn.acknowledge_armed(2)
        txn.begin_commit(11.1)
        txn.acknowledge_committed(1)
        txn.acknowledge_committed(2)
        txn.acknowledge_activated(1)
        txn.acknowledge_activated(2)
        txn.mark_activated()
        self.assertFalse(txn.acknowledge(1, True))


if __name__ == "__main__":
    unittest.main()
