/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   spacefillsearch.hpp
 * Author: mposypkin
 *
 * Created on March 7, 2017, 12:05 PM
 */


#ifndef SPACEFILLSEARCH_HPP
#define SPACEFILLSEARCH_HPP

#include <vector>
#include <functional>
#include <solver.hpp>
#include <mpproblem.hpp>
#include <pointgen/pointgenerator.hpp>
#include <common/vec.hpp>


namespace BBSEARCH {

    /**
     * The solver that uses some sort of point generator to fill the search space
     * with the subsequent running local search solver
     */
    template <typename FT> class SpaceFillSearch : public COMPI::Solver<FT> {
    public:

        /**
         * Determines stopping conditions
         * @param bestf - record value (best value found)
         * @param curf - function value on the last step
         * @param cnt - current step number
         * @return true if stop the search
         */
        typedef std::function<bool(FT bestf, FT curf, int n) > Stopper;

        class Watcher {
        public:

            /**
             * Called before local search
             * @param bestf record value (best value found)
             * @param inif initial value on the generated point
             * @param x generated point
             * @param cnt current step number
             */
            virtual void beforeLocalSearch(FT bestf, FT inif, const FT* x, int cnt) {
            }

            /**
             * Called after local search
             * @param bestf record value (best value found)
             * @param curf value found by the local search
             * @param x point found by the local search
             * @param cnt current step number
             */
            virtual void afterLocalSearch(FT bestf, FT curf, const FT* x, int cnt) {
            }

            /**
             * Called when the record is updated
             * @param prevf previous record value
             * @param newf new record value
             * @param prevx previous record value
             * @param newx point found
             * @param cnt current step number
             */
            virtual void update(FT prevf, FT bestf, const FT* prevx, const FT* newx, int cnt) {
            }

        };

        SpaceFillSearch(const COMPI::MPProblem<FT>& prob, snowgoose::PointGenerator<FT>& pointGenerator, COMPI::Solver<FT>& localSearch,
                const Stopper& stopper = [](FT f, FT curf, int n) {

                    return false;
                })
        : mProb(prob), mPointGenerator(pointGenerator), mLocalSearch(localSearch), mStopper(stopper), mWatcher(std::ref(mDefaultWatcher)) {
                    
        }

        bool search(FT* x, FT& v) override {
            const int n = mProb.mVarTypes.size();
            FT* tx = new double[n];
            int cnt = 0;
            while (mPointGenerator.getPoint(tx)) {
                cnt++;
                double tv = mProb.mObjectives.at(0)->func(tx);
                mWatcher.get().beforeLocalSearch(v, tv, tx, cnt);
                mLocalSearch.search(tx, tv);
                mWatcher.get().afterLocalSearch(v, tv, tx, cnt);
                if (tv < v) {
                    mWatcher.get().update(v, tv, x, tx, cnt);
                    snowgoose::VecUtils::vecCopy(n, tx, x);
                    v = tv;
                }
                if (mStopper(v, tv, cnt))
                    break;
            }
            delete [] tx;
        }

        /**
         * Updates watcher
         * @param watcher new watcher
         */
        void setWatcher(Watcher& watcher) {
            mWatcher = watcher;
        }
        
        std::string about() const override {
            return mPointGenerator.about() + mLocalSearch.about();
        }

        
    private:
        const COMPI::MPProblem<FT>& mProb;
        COMPI::Solver<FT>& mLocalSearch;
        snowgoose::PointGenerator<FT>& mPointGenerator;
        Stopper mStopper;        
        Watcher mDefaultWatcher;
        std::reference_wrapper<Watcher> mWatcher;
    };
}



#endif /* SPACEFILLSEARCH_HPP */

