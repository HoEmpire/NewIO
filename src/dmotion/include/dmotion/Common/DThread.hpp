#ifndef DTHREAD_H
#define DTHREAD_H

#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <stdio.h>

namespace Motion
{
  int get_thread_policy(pthread_attr_t *attr)
  {
    int policy;
    int rs = pthread_attr_getschedpolicy(attr,&policy);
    assert(rs==0);
    switch(policy)
    {
    case SCHED_FIFO:
      printf("policy= SCHED_FIFO\n");
      break;
    case SCHED_RR:
      printf("policy= SCHED_RR\n");
      break;
    case SCHED_OTHER:
      printf("policy=SCHED_OTHER\n");
      break;
    default:
      printf("policy=UNKNOWN\n");
      break;
    }
    return policy;
  }

  int get_thread_priority(pthread_attr_t *attr)
  {
    struct sched_param param;
    int rs = pthread_attr_getschedparam(attr,&param);
    assert(rs==0);
    printf("priority=%d\n",param.__sched_priority);
    return param.__sched_priority;
  }

  int set_thread_priority(pthread_attr_t *attr, int priority)
  {
    struct sched_param param;
    param.sched_priority = priority;
    pthread_attr_setschedparam(attr,&param);
    //pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
    return param.__sched_priority;
  }

  void set_thread_policy(pthread_attr_t *attr,int policy)
  {
    int rs = pthread_attr_setschedpolicy(attr,policy);
    assert(rs==0);
    get_thread_policy(attr);
  }

}
#endif
