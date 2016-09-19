#ifndef _NETWORK_COMMON_H
#define _NETWORK_COMMON_H

typedef struct {
    unsigned numGoodMessagesRecved;
    unsigned numCommErrors;
    unsigned numJSONRequestsRecved;
    unsigned numJSONResponsesRecved;
    unsigned numJSONRequestsSent;
    unsigned numJSONResponsesSent;
} MSGNetstats;

typedef struct {
    unsigned reading;
} MSGAdcReading;

typedef struct {
    char dummy;
} MSGQueryStats;

#endif
