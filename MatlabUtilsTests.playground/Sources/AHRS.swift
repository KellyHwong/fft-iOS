//
//  AHRS.swift
//  AHRS
//
//  Created by Kelly Hwong on 2017/12/4.
//  Copyright © 2017 HKUST. All rights reserved.
//


import Foundation
import Accelerate

public class AHRS {
    //%% Public properties
    public var SamplePeriod:Double = 1/256.0;
    public var Quaternion:[Double] = [1,0,0,0];     //% output quaternion describing the sensor relative to the Earth
    public var Kp:Double = 2;                     //% proportional gain
    public var Ki:Double = 0;                     //% integral gain
    public var KpInit:Double = 200;               //% proportional gain used during initialisation
    public var InitPeriod:Double = 5;             //% initialisation period in seconds
    
    //%% Private properties
    private var q:[Double] = [1,0,0,0];              //% internal quaternion describing the Earth relative to the sensor
    private var IntError:[Double] = [0,0,0];//TODO transpose        //% integral error
    private var KpRamped:Double;                   //% internal proportional gain used to ramp during initialisation
    
    public init(SamplePeriod:Double, Kp:Double, KpInit:Double) {
        self.SamplePeriod = SamplePeriod
        self.Kp = Kp
        self.KpInit = KpInit
        self.KpRamped = KpInit
    }
    
    public func UpdateIMU(Gyroscope:[Double], Accelerometer:[Double]) {
        let normOfAcc = norm(x:Accelerometer)
        assert(normOfAcc != 0.0)
        var Accelerometer = Accelerometer
        Accelerometer.enumerated().forEach { index, value in
            Accelerometer[index] = value / normOfAcc
        }
        var v:[Double] = [0,0,0]
        v[0] = 2*(self.q[1]*self.q[3] - self.q[0]*self.q[2])
        v[1] = 2*(self.q[0]*self.q[1] + self.q[2]*self.q[3])
        v[2] = self.q[0]*self.q[0] - self.q[1]*self.q[1] - self.q[2]*self.q[2] + self.q[3]*self.q[3]
        let error = cross(a:v, b:Accelerometer)
        //self.IntError = self.IntError + error
        for (i, iIntError) in self.IntError.enumerated() {
            self.IntError[i] = iIntError + error[i]
        }
        //% Apply feedback terms
        var Ref:[Double] = [0,0,0]
        for (i, iGyroscope) in Gyroscope.enumerated() {
            Ref[i] = iGyroscope - (self.Kp*error[i] + self.Ki*self.IntError[i])
        }
        //% Compute rate of change of quaternion
        var b:[Double] = [0,0,0,0]
        b[0] = 0
        b[1] = Ref[0]
        b[2] = Ref[1]
        b[3] = Ref[2]
        var pDot = self.quaternProd(a:self.q,b:b)
        for (i, ipDot) in pDot.enumerated() {
            pDot[i] = 0.5 * ipDot
        }
        for (i, _) in self.q.enumerated() {
            self.q[i] = self.q[i] + pDot[i] * self.SamplePeriod
        }
        
        let normOfQ = norm(x:self.q)
        self.q.enumerated().forEach { index, value in
            self.q[index] = value / normOfQ
        }
        //% Store conjugate
        self.Quaternion = self.quaternConj(q:self.q)
    }
    
    public func Reset() {
        self.KpRamped = self.KpInit //% start Kp ramp-down
        self.IntError = [0.0,0.0,0.0] //% reset integral terms
        self.q = [1,0,0,0] //% set quaternion to alignment
    }
    
    func quaternProd(a:[Double],b:[Double]) -> [Double] {
        var ab:[Double] = [0,0,0,0]
        ab[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
        ab[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
        ab[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
        ab[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
        return ab
    }
    
    func quaternConj(q:[Double]) -> [Double] {
        var qConj:[Double] = [0,0,0,0]
        qConj[0] = q[0]
        qConj[1] = -q[1]
        qConj[2] = -q[2]
        qConj[3] = -q[3]
        return qConj
    }
}
