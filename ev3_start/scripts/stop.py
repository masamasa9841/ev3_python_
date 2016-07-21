#!/usr/bin/env python
import rpyc
host="192.168.22.175"
conn=rpyc.classic.connect(host)
lego=conn.modules.ev3.lego
motorB=lego.Motor('A')
motorC=lego.Motor('D')
