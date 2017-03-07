/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   testmontecarlo.cpp
 * Author: mposypkin
 *
 * Created on March 7, 2017, 11:59 AM
 */

#include <iostream>
#include <limits>
#include <oneobj/contboxconstr/dejong.hpp>
#include <pointgen/randpointgen.hpp>
#include <methods/varcoordesc/varcoordesc.hpp>
#include "spacefillsearch.hpp"

class MyWatcher : public BBSEARCH::SpaceFillSearch<double>::Watcher {
    void update(double prevf, double bestf, const double* prevx, const double* newx, int cnt) override {
        std::cout << "Record update from  " << prevf << " to " << bestf << " on step " << cnt << "\n";
    }
};

/*
 * 
 */
int main(int argc, char** argv) {
    const int n = 10;
    const int num = 8;
    const int maxLocalSteps = 10;
    OPTITEST::DejongProblemFactory fact(n, -4, 8);
    COMPI::MPProblem<double> *mpp = fact.getProblem();
    snowgoose::RandomPointGenerator<double> gener(*(mpp->mBox), num);
    LOCSEARCH::VarCoorDesc<double> desc(*mpp, [&](double xdiff, double fdiff, const std::vector<double>& gran, double fval, int cnt) {
        if (cnt <= maxLocalSteps)
            return false;
        else
            return true;
    });
    BBSEARCH::SpaceFillSearch<double> sfSearch(*mpp, gener, desc);
    MyWatcher watcher;
    sfSearch.setWatcher(watcher);
    double x[n];
    double v = std::numeric_limits<double>::max();
    sfSearch.search(x, v);
    return 0;
}

