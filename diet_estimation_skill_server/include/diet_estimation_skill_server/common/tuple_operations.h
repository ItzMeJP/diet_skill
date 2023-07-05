#pragma once

/**\file grasp_estimation_skill.h
 * \brief Some operations with std::tuple<>
 *
 * @version 1.0
 * @author João Pedro Carvalho de Souza
 */

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   <macros>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>   </macros>  <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

// >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <includes>   <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
// std includes
#include <iostream>
#include <bits/stdc++.h>

namespace diet_estimation_skill {
    namespace tuple_operations {

        //implicit template otherwise I should implement all possibilities
        template <class T> bool sortBySecondAscending(const T & _a, const T & _b){
            return (std::get<1>(_a) < std::get<1>(_b));
        }

        template <class T> bool sortByThirdAscending(const T & _a, const T & _b){
            return (std::get<2>(_a) < std::get<2>(_b));
        }

        template <class T> bool sortByFourthAscending(const T & _a, const T & _b){
            return (std::get<3>(_a) < std::get<3>(_b));
        }

        template <class T> bool sortByFifthAscending(const T & _a, const T & _b){
            return (std::get<4>(_a) < std::get<4>(_b));
        }

        template <class T> bool sortBySixthAscending(const T & _a, const T & _b){
            return (std::get<5>(_a) < std::get<5>(_b));
        }

        template <class T> bool sortBySeventhAscending(const T & _a, const T & _b){
            return (std::get<6>(_a) < std::get<6>(_b));
        }


        template <class T> bool sortBySecondDescending(const T & _a, const T & _b){
            return (std::get<1>(_a) > std::get<1>(_b));
        }

        template <class T> bool sortByThirdDescending(const T & _a, const T & _b){
            return (std::get<2>(_a) > std::get<2>(_b));
        }

        template <class T> bool sortByFourthDescending(const T & _a, const T & _b){
            return (std::get<3>(_a) > std::get<3>(_b));
        }

        template <class T> bool sortByFifthDescending(const T & _a, const T & _b){
            return (std::get<4>(_a) > std::get<4>(_b));
        }

        template <class T> bool sortBySixthDescending(const T & _a, const T & _b){
            return (std::get<5>(_a) > std::get<5>(_b));
        }

        template <class T> bool sortBySeventhDescending(const T & _a, const T & _b){
            return (std::get<6>(_a) > std::get<6>(_b));
        }


/*
    bool sortByFourth(const tuple<int, int, int>& a,
                      const tuple<int, int, int>& b);
*/



    } /* namespace tuple_operations */
} /* namespace grasp_estimation_skill */

