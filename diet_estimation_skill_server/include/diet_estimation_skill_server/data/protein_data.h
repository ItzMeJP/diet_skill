//
// Created by joaopedro on 8/1/24.
//
#include <diet_estimation_skill_server/data/diet_estimation_data_base.h>


#ifndef DIET_ESTIMATION_SKILL_SERVER_PROTEIN_DATA_H
#define DIET_ESTIMATION_SKILL_SERVER_PROTEIN_DATA_H

namespace diet_estimation_skill {
    class ProteinData: public DietEstimationDataBase {
    public:
        //Constructor
        ProteinData(){};
        ~ProteinData(){};

        float foo_protein = 33.3;

    protected:

    };

}

#endif //DIET_ESTIMATION_SKILL_SERVER_PROTEIN_DATA_H
