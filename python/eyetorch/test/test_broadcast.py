# Copyright (c) Meta Platforms, Inc. and affiliates.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import unittest

import torch
from eyetorch.broadcast import check_size_and_broadcast, expand_to_match_ndim


class TestBroadcast(unittest.TestCase):
    def test_expand_to_match_ndim(self):

        a = torch.empty(2, 3, 4)
        b = torch.empty(4)

        b = expand_to_match_ndim(b, a)

        self.assertEqual(b.shape, torch.Size([1, 1, 4]))

    def test_check_size_and_broadcast(self):

        with self.subTest("Intrinsic size only"):
            a = torch.empty(6)
            b = torch.empty(3)

            a, b = check_size_and_broadcast(a, b, 6, 3)

            self.assertEqual(a.shape, torch.Size([6]))
            self.assertEqual(b.shape, torch.Size([3]))

        with self.subTest("A 1D, B 2D"):

            a = torch.empty(5)
            b = torch.empty(4, 2)

            a, b = check_size_and_broadcast(a, b, 5, 2)

            self.assertEqual(a.shape, torch.Size([4, 5]))
            self.assertEqual(b.shape, torch.Size([4, 2]))
            # self.assertTrue(a.is_contiguous())
            # self.assertTrue(b.is_contiguous())

        with self.subTest("A 2D, B 1D"):

            a = torch.empty(3, 6)
            b = torch.empty(2)

            a, b = check_size_and_broadcast(a, b, 6, 2)

            self.assertEqual(a.shape, torch.Size([3, 6]))
            self.assertEqual(b.shape, torch.Size([3, 2]))
            # self.assertTrue(a.is_contiguous())
            # self.assertTrue(b.is_contiguous())

        with self.subTest("A 2D, B 2D"):

            for a, b in zip(
                [torch.empty(3, 2), torch.empty(3, 2), torch.empty(1, 2)],
                [torch.empty(3, 7), torch.empty(1, 7), torch.empty(3, 7)],
            ):

                a, b = check_size_and_broadcast(a, b, 2, 7)

                self.assertEqual(a.shape, torch.Size([3, 2]))
                self.assertEqual(b.shape, torch.Size([3, 7]))
                # self.assertTrue(a.is_contiguous())
                # self.assertTrue(b.is_contiguous())

            with self.assertRaises(RuntimeError):
                check_size_and_broadcast(torch.empty(2, 4), torch.empty(3, 5), 4, 5)

        with self.subTest("A 3D, B 1D"):

            a = torch.empty(3, 6, 4)
            b = torch.empty(2)

            a, b = check_size_and_broadcast(a, b, 4, 2)

            self.assertEqual(a.shape, torch.Size([3, 6, 4]))
            self.assertEqual(b.shape, torch.Size([3, 6, 2]))
            # self.assertTrue(a.is_contiguous())
            # self.assertTrue(b.is_contiguous())

        with self.subTest("A 2D, B 3D"):

            for a, b in zip(
                [torch.empty(6, 3, 2), torch.empty(6, 3, 2), torch.empty(6, 1, 2)],
                [torch.empty(3, 7), torch.empty(1, 7), torch.empty(3, 7)],
            ):

                a, b = check_size_and_broadcast(a, b, 2, 7)

                self.assertEqual(a.shape, torch.Size([6, 3, 2]))
                self.assertEqual(b.shape, torch.Size([6, 3, 7]))
                # self.assertTrue(a.is_contiguous())
                # self.assertTrue(b.is_contiguous())

            with self.assertRaises(RuntimeError):
                check_size_and_broadcast(torch.empty(6, 3, 4), torch.empty(2, 5), 4, 5)
            with self.assertRaises(RuntimeError):
                check_size_and_broadcast(torch.empty(6, 3, 4), torch.empty(6, 5), 4, 5)
