#ifndef COROUTINE_H_
#define COROUTINE_H_

#include <stdint.h>

#define _RET_PASS      (0)
#define _RET_DONE      (1)
#define _RET_ERROR     (2)

typedef uint32_t task_t;

#define CR_BEGIN            static void* _RESUME_LABEL __attribute__((unused)) = 0; \
                            static task_t  _RET_VAL __attribute__((unused)) = 0;    \
                            static uint32_t _SLEEP_TIME __attribute__((unused)) = 0;\
                            do {                                                    \
                                if (_RESUME_LABEL) goto *_RESUME_LABEL;             \
                            } while (0)

#define _VAL_PASS(val)     do {__label__ resume; (_RESUME_LABEL) = &&resume; return (val); resume:;} while(0)
#define _VAL_RETURN(val)     do {(_RESUME_LABEL) = 0; return (val);} while(0)

#define CR_PASS             _VAL_PASS(_RET_PASS)
#define CR_RETURN           _VAL_RETURN(_RET_DONE)
#define CR_RESET            _VAL_RETURN(_RET_PASS)
#define CR_ERROR            _VAL_RETURN(_RET_ERROR)

#define CR_FAIL             (_RET_VAL == _RET_ERROR)

#define CR_POLL(pred)     do {                                                          \
                                do {CR_PASS;} while ((_RET_VAL=(pred)) == _RET_PASS);   \
                            } while (0)

#define CR_ENTER(pred)    do {                                                          \
                                while ((_RET_VAL=(pred)) == _RET_PASS) {CR_PASS;}       \
                            } while (0)

#define CR_SLEEP(ms)      for (_SLEEP_TIME = millis();              \
                                _SLEEP_TIME + ms + 1 > millis();)   \
                            {CR_PASS;}

#endif /* COROUTINE_H_*/
