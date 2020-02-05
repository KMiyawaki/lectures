#!/usr/bin/env python
# -*- coding: utf-8 -*-
from datetime import datetime
from employee import Employee


class Leader(Employee):
    def __init__(self, id="x18-000", name="工大 太郎", birth=datetime(1980, 12, 1), join=datetime(2005, 4, 1), team="カスタマーサポート", salary=230000):
        Employee.__init__(self, id, name, birth, join, team, salary)
