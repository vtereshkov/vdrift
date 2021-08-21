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

#ifndef _UMKAWRAPPER_H
#define _UMKAWRAPPER_H

#include "umka_api.h"


class Umka
{
public:

    Umka(const char *fileName, const char *sourceString = nullptr, int storageSize = 1024 * 1024, int stackSize = 1024 * 1024,
         const char *locale = nullptr, int argc = 0, char **argv = nullptr):
             handle(umkaAlloc()),
             initOK(false),
             compileOK(false),
             runOK(false),
             mainReturned(false)
    {
        initOK = handle && umkaInit(handle, fileName, sourceString, storageSize, stackSize, locale, argc, argv);
    }

    bool compile()
    {
        compileOK = initOK && umkaCompile(handle);
        return compileOK;
    }

    bool run()
    {
        runOK = compileOK && umkaRun(handle);
        if (compileOK)
            mainReturned = true;
        return runOK;
    }

    bool call(int entryOffset, int numParamSlots, UmkaStackSlot *params, UmkaStackSlot *result)
    {
        runOK = compileOK && umkaCall(handle, entryOffset, numParamSlots, params, result);
        return runOK;
    }

    bool addModule(const char *fileName, const char *sourceString)
    {
        if (!initOK)
            return false;
        umkaAddModule(handle, fileName, sourceString);
        return true;
    }

    bool addFunc(const char *name, UmkaExternFunc entry)
    {
        if (!initOK)
            return false;
        umkaAddFunc(handle, name, entry);
        return true;
    }

    int getFunc(const char *moduleName, const char *funcName)
    {
        if (!initOK)
            return -1;
        return umkaGetFunc(handle, moduleName, funcName);
    }

    bool ok() const
    {
        return compileOK && runOK;
    }

    std::string getError()
    {
        UmkaError error;
        std::string res = "";

        if (!compileOK)
        {
            umkaGetError(handle, &error);
            res = "Umka error " + std::string(error.fileName) + "(" + std::to_string(error.line) + ", " + std::to_string(error.pos) + "): " + std::string(error.msg);
        }
        else if (!runOK)
        {
            umkaGetError(handle, &error);
            res = "\nUmka runtime error " + std::string(error.fileName) + "(" + std::to_string(error.line) + "): " + std::string(error.msg);
            res += "\nStack trace:\n";

            for (int depth = 0; depth < 10; depth++)
            {
                char fnName[UMKA_MSG_LEN + 1];
                int fnOffset;

                if (!umkaGetCallStack(handle, depth, &fnOffset, fnName, UMKA_MSG_LEN + 1))
                    break;

                res += std::to_string(fnOffset) + ": " + std::string(fnName) + "\n";
            }
        }

        return res;
    }

    ~Umka()
    {
        // Run GC for global objects
        if (compileOK && !mainReturned)
            run();

        if (handle)
            umkaFree(handle);
    }

private:

    void *handle;
    bool initOK;
    bool compileOK;
    bool runOK;
    bool mainReturned;
};

#endif


