#    Copyright 2022 Christoph Hellmann Santos
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

import os
import time
import pytest
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import threading
import rclpy
from canopen_utils.launch_test_node import LaunchTestNode
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
import unittest
from canopen_interfaces.srv import CORead, COWrite
from canopen_interfaces.msg import COData


def _change_state(node, transition_id):
    """Trigger a lifecycle transition via the lifecycle_manager."""
    req = ChangeState.Request(transition=Transition(id=transition_id))
    res = ChangeState.Response(success=True)
    node.call_service("/lifecycle_manager/change_state", ChangeState, req, res)


@pytest.mark.rostest
def generate_test_description():

    launch_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("canopen_tests"), "launch"),
                "/proxy_lifecycle_setup.launch.py",
            ]
        )
    )

    ready_to_test = launch.actions.TimerAction(
        period=5.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return (LaunchDescription([launch_desc, ready_to_test]), {})


class TestLifecycle(unittest.TestCase):
    def run_node(self):
        while self.ok:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    @classmethod
    def setUp(cls):
        cls.ok = True
        rclpy.init()
        cls.node = LaunchTestNode()
        cls.thread = threading.Thread(target=cls.run_node, args=[cls])
        cls.thread.start()

    @classmethod
    def tearDown(cls):
        cls.ok = False
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_configure_unconfigure(self):
        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        _change_state(self.node, Transition.TRANSITION_CLEANUP)

    def test_full_cycle(self):
        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_CLEANUP)
        print("*************************CLEANUP SUCCESSFUL*************************")

        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_CLEANUP)
        print("*************************CLEANUP SUCCESSFUL*************************")


class TestSDO(unittest.TestCase):
    def run_node(self):
        while self.ok:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    @classmethod
    def setUp(cls):
        cls.ok = True
        rclpy.init()
        cls.node = LaunchTestNode()
        cls.thread = threading.Thread(target=cls.run_node, args=[cls])
        cls.thread.start()

    @classmethod
    def tearDown(cls):
        cls.ok = False
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _sdo_write_read_both(self, index, subindex, data):
        """Write data to (index, subindex) on both proxies, then read it back."""
        write_req = COWrite.Request()
        write_req.index = index
        write_req.subindex = subindex
        write_req.data = data
        write_res = COWrite.Response()
        write_res.success = True

        read_req = CORead.Request()
        read_req.index = index
        read_req.subindex = subindex
        read_res = CORead.Response()
        read_res.success = True
        read_res.data = data

        self.node.call_service("proxy_device_1/sdo_write", COWrite, write_req, write_res)
        self.node.call_service("proxy_device_2/sdo_write", COWrite, write_req, write_res)
        self.node.call_service("proxy_device_1/sdo_read", CORead, read_req, read_res)
        self.node.call_service("proxy_device_2/sdo_read", CORead, read_req, read_res)

    def _sdo_read_both(self, index, subindex, expected):
        """Read (index, subindex) on both proxies, expecting the given value."""
        read_req = CORead.Request()
        read_req.index = index
        read_req.subindex = subindex
        read_res = CORead.Response()
        read_res.success = True
        read_res.data = expected

        self.node.call_service("proxy_device_1/sdo_read", CORead, read_req, read_res)
        self.node.call_service("proxy_device_2/sdo_read", CORead, read_req, read_res)

    def test_full_cycle_sdo(self):
        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        self._sdo_write_read_both(0x4000, 0, 100)

        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_CLEANUP)
        print("*************************CLEANUP SUCCESSFUL*************************")

        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        self._sdo_read_both(0x4000, 0, 100)

        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_CLEANUP)
        print("*************************CLEANUP SUCCESSFUL*************************")

    def test_full_cycle_sdo_64(self):
        """Tests UNSIGNED64 and INTEGER64 SDO writes/reads in the lifecycle context."""
        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)

        # UNSIGNED64 round-trip on 0x4004 sub10.
        self._sdo_write_read_both(0x4004, 10, 0xCAFE00000000F00D)

        # INTEGER64 round-trip on 0x4004 sub11 (verify sign extension).
        # CORead/COWrite.data is uint64 on the wire; encode signed value.
        value = -(1 << 60)
        sign_extended = value & ((1 << 64) - 1)
        self._sdo_write_read_both(0x4004, 11, sign_extended)

        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        _change_state(self.node, Transition.TRANSITION_CLEANUP)

    def test_full_cycle_sdo_48(self):
        """Tests UNSIGNED48 and INTEGER48 SDO writes/reads in the lifecycle context."""
        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)

        # UNSIGNED48 round-trip on 0x4004 sub8.
        self._sdo_write_read_both(0x4004, 8, 0xCAFE0000F00D)

        # INTEGER48 round-trip on 0x4004 sub9 (verify sign extension).
        value = -(1 << 40)
        sign_extended = value & ((1 << 64) - 1)
        self._sdo_write_read_both(0x4004, 9, sign_extended)

        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        _change_state(self.node, Transition.TRANSITION_CLEANUP)


class TestPDO(unittest.TestCase):
    def run_node(self):
        while self.ok:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    @classmethod
    def setUp(cls):
        cls.ok = True
        rclpy.init()
        cls.node = LaunchTestNode()
        cls.thread = threading.Thread(target=cls.run_node, args=[cls])
        cls.thread.start()

    @classmethod
    def tearDown(cls):
        cls.ok = False
        cls.thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_full_cycle_sdo(self):
        _change_state(self.node, Transition.TRANSITION_CONFIGURE)
        print("*************************CONFIGURE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_ACTIVATE)
        print("*************************ACTIVATE SUCCESSFUL*************************")

        msg = COData()
        msg.index = 0x4000
        msg.subindex = 0
        msg.data = 200

        pub_msg = COData()
        pub_msg.index = 0x4001
        pub_msg.subindex = 0
        pub_msg.data = 200

        with self.node.expect_message("proxy_device_1/rpdo", COData, pub_msg) as waiter:
            self.node.publish_message(
                "proxy_device_1/tpdo", COData, msg, wait_for_subscribers_timeout=2.0
            )
            got = waiter.wait(timeout=2.5)

        # Run deactivate/cleanup regardless of round-trip success so the
        # next test starts from a known unconfigured state. Assert PDO
        # round-trip last so a PDO failure doesn't strand the device in
        # ACTIVE and cascade into later tests.
        _change_state(self.node, Transition.TRANSITION_DEACTIVATE)
        print("*************************DEACTIVATE SUCCESSFUL*************************")
        _change_state(self.node, Transition.TRANSITION_CLEANUP)
        print("*************************CLEANUP SUCCESSFUL*************************")

        self.assertTrue(got, f"Did not receive expected COData {pub_msg} on proxy_device_1/rpdo")
