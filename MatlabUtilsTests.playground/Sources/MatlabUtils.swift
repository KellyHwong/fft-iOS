//
//  MatlabUtils.swift
//  MatlabUtils
//
//  Created by Kelly Hwong on 2017/11/29.
//  Copyright © 2017 HKUST. All rights reserved.
//

import Foundation

extension RangeReplaceableCollection {
    public mutating func resize(_ size: IndexDistance, fillWith value: Iterator.Element) {
        let c = count
        if c < size {
            append(contentsOf: repeatElement(value, count: c.distance(to: size)))
        } else if c > size {
            let newEnd = index(startIndex, offsetBy: size)
            removeSubrange(newEnd ..< endIndex)
        }
    }
}

public func filter(a:Array<Double>, b:Array<Double>, x:Array<Double>, zi:Array<Double>) -> Array<Double> {
    assert(a.count == b.count)
    var zi = zi
    let filter_order = a.count
    zi.resize(filter_order, fillWith: 0)
    var y = [Double](repeating: 0.0, count: x.count)
    
    for i in 0...x.count-1 {
        var order = filter_order - 1
        while(order>0){
            if (i >= order) {
                zi[order - 1] = b[order] * x[i - order] - a[order] * y[i - order] + zi[order]
            }
            order = order - 1
        }
        y[i] = b[0] * x[i] + zi[0]
    }
    zi.resize(filter_order - 1, fillWith: 0)
    return y
}

public func filter(a:Array<Double>, b:Array<Double>, x:Array<Double>) -> Array<Double> {
    var y = [Double](repeating: 0.0, count: x.count)
    
    y[0] = b[0] * x[0];
    for i in 1...b.count-1 {
        y[i] = 0.0
        for j in 0...i {
            y[i] += b[j] * x[i-j]
        }
        for j in 0...i-1 {
            y[i] -= a[j+1] * y[i-j-1];
        }
    }
    
    for i in b.count...x.count-1 {
        y[i] = 0.0
        for j in 0...b.count-1 {
            y[i] += b[j] * x[i-j]
        }
        for j in 0...b.count-2 {
            y[i] -= a[j+1] * y[i-j-1];
        }
    }
    
    return y
}

public func filtfilt(a:Array<Double>, b:Array<Double>, x:Array<Double>) -> Array<Double> {
    
    let border_size = 3*(a.count-1)
    
    assert(a.count == b.count && x.count > 2*border_size)
    
    // Reduce boundary effect - grow the signal with its inverted replicas on both edges
    var xx = [Double](repeating: 0.0, count: x.count + 2*border_size)
    
    for i in 0...border_size-1 {
        xx[i] = 2*x[0] - x[border_size-i] //x[border_size-i+1]
        xx[xx.count-i-1] = 2*x.last! - x[x.count-border_size+i-1]
    }
    for i in 0...x.count-1 {
        xx[i+border_size] = x[i];
    }
    
    //var a = a1; var b = b1;
    //var nfilt = 2;
    //var rows = 1; var cols = 1;
    let vals = 1+a[2-1];
    let rhs  = b[2-1] - b[1-1]*a[2-1]
    let zi:[Double]   = [rhs / vals]
    //print("zi:\(zi)")
    //print(xx)
    var zi_multiplied = zi.map { $0 * xx[1-1] }
    //print("zi_multiplied:\(zi_multiplied)")
    // one-way filter
    let firstPass = filter(a:a,b:b,x:xx,zi:zi_multiplied)
    //print("firstPass:\(firstPass)")
    
    // reverse the series
    let rev = Array(firstPass.reversed())
    //print("rev:\(rev)")
    
    // filter again
    zi_multiplied = zi.map { $0 * rev[1-1] }
    //print("zi_multiplied:\(zi_multiplied)")
    let secondPass = filter(a:a, b:b, x:rev, zi:zi_multiplied)
    //print("secondPass:\(secondPass)")
    
    // return a stripped series, reversed back
    let secondPassRev = Array(secondPass.reversed())
    //print("secondPassRev:\(secondPassRev)")
    
    let secondPassStripped = Array(secondPassRev[border_size..<secondPassRev.count-border_size])
    
    return secondPassStripped
}

public func filtfilt_backup(a:Array<Double>, b:Array<Double>, x:Array<Double>) -> Array<Double> {
    
    let border_size = 3*a.count
    
    assert(a.count == b.count && x.count > 2*border_size)
    
    // Reduce boundary effect - grow the signal with its inverted replicas on both edges
    var xx = [Double](repeating: 0.0, count: x.count + 2*border_size)
    //return secondPass
    
    for i in 0...border_size-1 {
        xx[i] = 2*x[0] - x[border_size-i-1]
        xx[xx.count-i-1] = 2*x.last! - x[x.count-border_size+i]
    }
    for i in 0...x.count-1 {
        xx[i+border_size] = x[i];
    }
    
    // one-way filter
    let firstPass = filter(a:a,b:b,x:xx)
    
    // reverse the series
    let rev = Array(firstPass.reversed())
    
    // filter again
    let secondPass = filter(a:a, b:b, x:rev)
    
    // return a stripped series, reversed back
    let secondPassRev = Array(secondPass.reversed())
    let secondPassStripped = Array(secondPassRev[border_size..<secondPassRev.count-border_size])
    
    return secondPassStripped
}

public func norm(x:[Double]) -> Double{
    var sqr_sum:Double = 0;
    for i in 0...x.count-1 {
        sqr_sum = sqr_sum + x[i] * x[i]
    }
    return sqrt(sqr_sum)
}

public func cross(a:[Double],b:[Double]) -> [Double] {
    var c:[Double] = [0,0,0]
    c[0] = a[1]*b[2] - a[2]*b[1]
    c[1] = a[2]*b[0] - a[0]*b[2]
    c[2] = a[0]*b[1] - a[1]*b[0]
    return c
}

public func mean(a:[Double]) -> Double {
    var sum:Double = 0
    for (_, ia) in zip(a.indices, a) {
        sum = sum + ia
    }
    return sum/Double(a.count)
}

public func deg2rad(D:Double) -> Double {
    return D * Double.pi / 180.0
}

public func quaternProd(a:[Double],b:[Double]) -> [Double] {
    var ab:[Double] = [0,0,0,0]
    ab[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
    ab[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
    ab[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
    ab[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
    return ab
}

public func quaternProd(a:[[Double]],b:[[Double]]) -> [[Double]] {
    var ab = a
    for i in 0...a.count-1 {
        ab[i][0] = a[i][0]*b[i][0]-a[i][1]*b[i][1]-a[i][2]*b[i][2]-a[i][3]*b[i][3];
        ab[i][1] = a[i][0]*b[i][1]+a[i][1]*b[i][0]+a[i][2]*b[i][3]-a[i][3]*b[i][2];
        ab[i][2] = a[i][0]*b[i][2]-a[i][1]*b[i][3]+a[i][2]*b[i][0]+a[i][3]*b[i][1];
        ab[i][3] = a[i][0]*b[i][3]+a[i][1]*b[i][2]-a[i][2]*b[i][1]+a[i][3]*b[i][0];
    }
    return ab
}

public func quaternConj(q:[Double]) -> [Double] {
    var qConj:[Double] = [0,0,0,0]
    qConj[0] = q[0]
    qConj[1] = -q[1]
    qConj[2] = -q[2]
    qConj[3] = -q[3]
    return qConj
}

public func quaternConj(q:[[Double]]) -> [[Double]] {
    var qConj = q
    for i in 0...q.count-1 {
        qConj[i][0] = q[i][0]
        qConj[i][1] = -q[i][1]
        qConj[i][2] = -q[i][2]
        qConj[i][3] = -q[i][3]
    }
    return qConj
}

public func quaternRotate(v:[[Double]], q:[[Double]]) -> [[Double]] {
    var v0 = v
    for i in 0...v0.count-1 {
        v0[i].insert(0.0, at:0)
    }
    let qConj = quaternConj(q:q)
    let qProd = quaternProd(a:q, b:v0)
    let v0XYZ = quaternProd(a:qProd, b:qConj)
    var v = v0XYZ
    for i in 0...v.count-1 {
        v[i].remove(at:0)
    }
    return v
}

public func diff(X:[Int]) -> [Int] {
    var Y:[Int] = []
    for i in 1...X.count-1 {
        Y.append(X[i]-X[i-1])
    }
    return Y
}

public func find(X:[Int],num:Int) -> [Int] {
    var k:[Int] = []
    for i in 0...X.count-1 {
        if X[i] == num {
            k.append(i)
        }
    }
    return k
}
