#!/usr/bin/env python
# coding:utf-8
import casadi as ca
import numpy as np

X = ca.SX.sym('x', 2, 1)
A = ca.MX.sym('A', 2, 2)
T = ca.MX(2, 2)
T[0, 0] = A[0, 0]
print(type(T))
# y = A @ X
# print(y)