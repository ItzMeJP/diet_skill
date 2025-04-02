//
// Created by joaopedro on 8/1/24.
//

#include <string>

#ifndef DIET_ESTIMATION_SKILL_SERVER_DIET_ESTIMATION_DATA_BASE_CPP_H
#define DIET_ESTIMATION_SKILL_SERVER_DIET_ESTIMATION_DATA_BASE_CPP_H



namespace diet_estimation_skill {
    class DietEstimationDataBase {
    public:
        //Constructor
        DietEstimationDataBase(){};
        virtual ~DietEstimationDataBase() = default;


        std::string foo_base = "foo";

    protected:

    };

}









#endif //DIET_ESTIMATION_SKILL_SERVER_DIET_ESTIMATION_DATA_BASE_CPP_H
