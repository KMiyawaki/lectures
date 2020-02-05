#!/usr/bin/python

import Task
import sys
import os

def work():
    dir = "task_list"
    files = os.listdir(dir)
    tasks = []
    for t in files:
        tasks.append(Task.Task(dir + "/" + t))

    while(True):
        for t in tasks:
            t.clear()
        print "Input setence : ",
        s = sys.stdin.readline().rstrip()
        if s == '':
            break
        for t in tasks:
            t.fill(s)
        maxr = 0; maxt = []
        for t in tasks:
            r = t.fill_rate()
            if maxr < r:
                maxr = r
                maxt = [t]
            else:
                if maxr == r:
                    maxt.append(t)
        if len(maxt) > 1:
            maxslot = 0
            maxtt = None
            for t in maxt:
                r = t.num_slot()
                if maxslot < r:
                    maxslot = r
                    maxtt = t
        else:
            maxtt = maxt[0]
        print "Result:"
        print "  TaskName : %s" % maxtt.task_name()
        res = maxtt.result()
        for s in res.keys():
            print "    %s : %s" % ( s, res[s])


if __name__ == "__main__":
    work()
