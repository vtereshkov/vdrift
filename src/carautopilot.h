/************************************************************************/
/*                                                                      */
/* This file is part of VDrift.                                         */
/*                                                                      */
/* VDrift is free software: you can redistribute it and/or modify       */
/* it under the terms of the GNU General Public License as published by */
/* the Free Software Foundation, either version 3 of the License, or    */
/* (at your option) any later version.                                  */
/*                                                                      */
/* VDrift is distributed in the hope that it will be useful,            */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of       */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        */
/* GNU General Public License for more details.                         */
/*                                                                      */
/* You should have received a copy of the GNU General Public License    */
/* along with VDrift.  If not, see <http://www.gnu.org/licenses/>.      */
/*                                                                      */
/************************************************************************/

#ifndef _CARAUTOPILOT_H
#define _CARAUTOPILOT_H

#include "umkawrapper.h"
#include "roadpatch.h"


class CarAutopilot
{
public:

    CarAutopilot(const std::string &scriptFile, std::ostream &errorOut):
        errorStream(errorOut),
        scriptName(scriptFile),
        umka(nullptr),
        umkaUpdateFunc(-1),
        engaged(false),
        inputs(CarInput::INVALID, 0.0f)
    {
    }

    ~CarAutopilot()
    {
        delete umka;
    }

    bool Reset()
    {
        if (umka)
            delete umka;

        umka = new Umka(scriptName.c_str());
        umkaUpdateFunc = -1;
        engaged = false;
        inputs = std::vector<float>(CarInput::INVALID, 0.0f);

        umka->addFunc("getPatchData", CarAutopilot::getPatchData);

        if (!umka->compile())
        {
            errorStream << umka->getError() << std::endl;
            return false;
        }

        umkaUpdateFunc = umka->getFunc(nullptr, "update");
        if (umkaUpdateFunc < 0)
        {
            errorStream << "Umka error: update() not found" << std::endl;
            return false;
        }

        return true;
    }

    bool Update(double dt, const CarDynamics &car)
    {
        if (!engaged)
            return false;

        if (umkaUpdateFunc < 0)
            return false;

        const btVector3 &pos = car.GetCenterOfMass();
        const btVector3 &vel = car.GetVelocity();
        const btMatrix3x3 att(car.GetOrientation());
        const btVector3 &rate = car.GetAngularVelocity();
        const RoadPatch *patch = GetCurrentPatch(car);

        const double posArray[] = {pos.x(), pos.y(), pos.z()};
        const double velArray[] = {vel.x(), vel.y(), vel.z()};

        double attMatrix[3][3];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                attMatrix[i][j] = att[i][j];

        const double rateArray[] = {rate.x(), rate.y(), rate.z()};

        UmkaStackSlot param[] = {{.ptrVal  = (int64_t)inputs.data()},
                                 {.ptrVal  = (int64_t)patch},
                                 {.ptrVal  = (int64_t)&rateArray},
                                 {.ptrVal  = (int64_t)&attMatrix},
                                 {.ptrVal  = (int64_t)&velArray},
                                 {.ptrVal  = (int64_t)&posArray},
                                 {.realVal = dt}};
        UmkaStackSlot result = {0};

        if (!umka->call(umkaUpdateFunc, sizeof(param) / sizeof(param[0]), param, &result))
        {
            errorStream << umka->getError() << std::endl;
            engaged = false;
            return false;
        }

        if (!result.intVal)
        {
            errorStream << "Umka: update() failed" << std::endl;
            engaged = false;
            return false;
        }

        return true;
    }

    const std::vector<float> &GetInputs()
    {
        return inputs;
    }

    void Engage(bool on)
    {
        if (!engaged && on)
            Reset();
        engaged = on;
    }

    bool IsEngaged() const
    {
        return engaged;
    }

private:

    std::ostream &errorStream;
    const std::string scriptName;
	Umka *umka;
	int umkaUpdateFunc;
	bool engaged;
	std::vector<float> inputs;

	const RoadPatch *GetCurrentPatch(const CarDynamics &car)
    {
        const RoadPatch *patch = car.GetWheelContact(FRONT_LEFT).GetPatch();
        if (!patch)
            patch = car.GetWheelContact(FRONT_RIGHT).GetPatch();
        return patch;
    }

    static void getPatchData(UmkaStackSlot *params, UmkaStackSlot *result)
    {
        struct RoadPatchData
        {
            double corners[4][3];
        };

        RoadPatch *patch = (RoadPatch *)params[2].ptrVal;
        int64_t lookahead = params[1].intVal;
        RoadPatchData *data = (RoadPatchData *)params[0].ptrVal;

        for (int i = 0; i < lookahead && patch; i++)
            patch = patch->GetNextPatch();

        if (!patch)
        {
            result->intVal = false;
            return;
        }

        Vec3 corners[] = {patch->GetPoint(0, 0), patch->GetPoint(0, 3),
                          patch->GetPoint(3, 0), patch->GetPoint(3, 3)};

        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 3; j++)
                data->corners[i][j] = corners[i][j];

        result->intVal = true;
    }


};



#endif

