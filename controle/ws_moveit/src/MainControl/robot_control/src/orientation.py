#!/usr/bin/env python

import math

class orientation:
    __xPosition = 0
    __yPosition = 0
    __zPosition = 0
    __xQuaternion = 0
    __yQuaternion = 0
    __zQuaternion = 0
    __wQuaternion = 0

    def __init__(self, xPosition=-1, yPosition=-1, zPosition=-1, xQuaternion=-1, yQuaternion=-1, zQuaternion=-1, wQuaternion=-1) :
        self.xPosition = xPosition
        self.yPosition = yPosition
        self.zPosition = zPosition
        self.xQuaternion = xQuaternion
        self.yQuaternion = yQuaternion
        self.zQuaternion = zQuaternion
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

    def SetQuaternionX(self, xQuaternion):
        self.xQuaternion = xQuaternion

    def GetQuaternionX(self):
        return self.xQuaternion

    def SetQuaternionY(self, yQuaternion):
        self.yQuaternion = yQuaternion

    def GetQuaternionY(self):
        return self.yQuaternion

    def SetQuaternionZ(self, zQuaternion):
        self.zQuaternion = zQuaternion

    def GetQuaternionZ(self):
        return self.zQuaternion

    def SetQuaternionW(self, wQuaternion):
        self.wQuaternion = wQuaternion

    def GetQuaternionW(self):
        return self.wQuaternion