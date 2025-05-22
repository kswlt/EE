#include <stdio.h>
#include <time.h>

// ����״̬ö��
typedef enum {
    STATE_NORMAL,
    STATE_SUSPECT,
    STATE_CONFIRMED,
    STATE_HANDLING
} JamState;

// ȫ�ֱ���
JamState currentState = STATE_NORMAL;
time_t stateStartTime;
float dialMotorTorque;

// ��ȡ���̵����ǰŤ�ص���
float getDialMotorTorque() {
    // ����������ʵ�ʻ�ȡŤ�ص����Ĵ���
    return dialMotorTorque;
}

// ״̬��������
void jamStateMachine() {
    time_t currentTime = time(NULL);
    double elapsedTime = difftime(currentTime, stateStartTime);

    switch (currentState) {
        case STATE_NORMAL:
            if (getDialMotorTorque() > 9.5) {
                currentState = STATE_SUSPECT;
                stateStartTime = currentTime;
                printf("Switch to STATE_SUSPECT\n");
            }
            break;
        case STATE_SUSPECT:
            if (elapsedTime > 0.3) {
                currentState = STATE_CONFIRMED;
                stateStartTime = currentTime;
                printf("Switch to STATE_CONFIRMED\n");
                // ���̵���л����Ƕȱջ������ݵ�ǰ�Ƕ�ֵ�ز�
                printf("Switch dial motor to angle closed-loop and reverse\n");
            } else if (getDialMotorTorque() <= 9.5) {
                currentState = STATE_NORMAL;
                printf("Switch back to STATE_NORMAL\n");
            }
            break;
        case STATE_CONFIRMED:
            currentState = STATE_HANDLING;
            stateStartTime = currentTime;
            printf("Switch to STATE_HANDLING\n");
            break;
        case STATE_HANDLING:
            if (elapsedTime > 0.2) {
                currentState = STATE_NORMAL;
                printf("Switch back to STATE_NORMAL\n");
            }
            break;
    }
}



    