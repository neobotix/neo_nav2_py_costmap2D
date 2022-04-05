#! /usr/bin/env python3

class LineIterator():

    def __init__(self, x0, y0, x1, y1):
        self.x0_ = x0
        self.y0_ = x0
        self.x1_ = x1
        self.y1_ = y1
        self.x_ = x0
        self.y_ = y0

        self.delta_x_ = abs(self.x1_ - self.x0_)
        self.delta_y_ = abs(self.y1_ - self.y0_)
        self.curpixel_ = 0
        self.xinc1_, self.xinc2_, self.yinc1_, self.yinc2_ = 0, 0, 0, 0
        self.num_, self.den_ = 0, 0
        self.numadd_ = 0

        if (self.x1_ >= x0):
            self.xinc1_ = 1
            self.xinc2_ = 1
        else:
            self.xinc1_ = -1
            self.xinc2_ = -1

        if (y1 >= y0):
            self.yinc1_ = 1
            self.yinc2_ = 1
        else:
            self.yinc1_ = -1
            self.yinc2_ = -1

        if (self.delta_x_ >= self.delta_y_):
            self.xinc1_ = 0
            self.yinc2_ = 0
            self.den_ = self.delta_x_
            self.num_ = self.delta_x_ / 2
            self.numadd_ = self.delta_y_
            self.numpixels_ = self.delta_x_
        else:
            self.xinc2_ = 0
            self.yinc1_ = 0
            self.den_ = self.delta_y_
            self.num_ = self.delta_y_ / 2
            self.numadd_ = self.delta_x_
            self.numpixels_ = self.delta_y_

    def isValid(self):
        return self.curpixel_ <= self.numpixels_

    def advance(self):
        self.num_ = self.numadd_
        if (self.num_ >= self.den_):
            self.num_ -= self.den_
            self.x_ += self.xinc1_
            self.y_ += self.yinc1_
        self.x_ += self.xinc2_
        self.y_ += self.yinc2_
        self.curpixel_+=1

    def getX(self):
        return self.x_

    def getY(self):
        return self.y_

    def getX0(self):
        return self.x0_

    def getY0(self):
        return self.y0_

    def getX1(self):
        return self.x1_

    def getY1(self):
        return self.y1_
