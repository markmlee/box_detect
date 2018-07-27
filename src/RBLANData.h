#ifndef RBLANDATA_H
#define RBLANDATA_H

#include "../RBSharedMemory.h"
#include "../UserSharedMemory.h"

typedef struct __LAN_STRUCT_GUI2PODO__
{
    USER_COMMAND            UserCMD;
    GUI2MOTION              UserG2M;
} LAN_GUI2PODO,*pLAN_GUI2PODO;

typedef struct __LAN_STRUCT_PODO2GUI__
{
    RBCORE_SHM      CoreData;
    MOTION2GUI      UserM2G;
} LAN_PODO2GUI, *pLAN_PODO2GUI;


typedef struct __LAN_STRUCT_PODO2VISION__
{
    RBCORE_SHM      CoreData;
} LAN_PODO2VISION, *pLAN_PODO2VISION;


#endif // RBLANDATA_H
