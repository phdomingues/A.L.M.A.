#!/usr/bin/env python

import math

class Orientation:
    __xPosition = 0
    __yPosition = 0
    __zPosition = 0
    __wQuaternion = 0

    def __init__(self, xPosition=-1, yPosition=-1, zPosition=-1, wQuaternion=-1) :
        self.xPosition = xPosition
        self.yPosition = yPosition
        self.zPosition = zPosition
        self.wQuaternion = wQuaternion

    #Setters & Getters
    def SetPositionX(self, xPosition):
        self.xPosition = xPosition

    def GetPositionX(self):
        return self.xPosition

    def SetPositionY(self, yPosition):
        self.yPosition = yPosition

    def GetPositionY(self):
        return self.yPosition

    def SetPositionZ(self, zPosition):
        self.zPosition = zPosition

    def GetPositionZ(self):
        return self.zPosition

    def SetwQuaternionW(self, wQuaternion):
        self.wQuaternion = wQuaternion

    def GetwQuaternionW(self):
        return self.wQuaternion