#!/usr/bin/env python
# -*- coding: utf-8 -*-
from datetime import datetime


class Employee(object):
    def __init__(self, id="x18-000", name="工大 太郎", birth=datetime(1980, 12, 1), join=datetime(2005, 4, 1), team="カスタマーサポート"):
        self.id = id
        self.name = name
        self.birth = birth  # 誕生日
        self.join = join  # 入社日
        self.team = team  # 部署

    def get_age(self, year):
        return year - self.birth.year  # かなり簡略化している。
