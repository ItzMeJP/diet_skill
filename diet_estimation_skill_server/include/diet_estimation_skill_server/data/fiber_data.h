//
// Created by joaopedro on 8/1/24.
//

#include <diet_estimation_skill_server/data/diet_estimation_data_base.h>


#ifndef DIET_ESTIMATION_SKILL_SERVER_FIBER_DATA_H
#define DIET_ESTIMATION_SKILL_SERVER_FIBER_DATA_H


namespace diet_estimation_skill {
    class FiberData: public DietEstimationDataBase {
    public:
        //Constructor
        FiberData(){};
        ~FiberData(){};

        bool foo_fiber = true;

    protected:

    };

}

#endif //DIET_ESTIMATION_SKILL_SERVER_FIBER_DATA_H
