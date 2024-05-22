/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 1
#define PRIORITY_TOPENCOMROBOT 4
#define PRIORITY_TMOVE 6
#define PRIORITY_TSENDTOMON 3
#define PRIORITY_TRECEIVEFROMMON 4
#define PRIORITY_TSTARTROBOT 5
#define PRIORITY_TCAMERA 11
#define PRIORITY_TCAMERA_OFF 8
#define PRIORITY_TCAMERA_ON 9
#define PRIORITY_TBATTERY 6
#define PRIORITY_TCONNEXIONTOROBOTLOST 4
#define PRIORITY_TMONITORCONNEXIONLOST 4
#define PRIORITY_TSTOPROBOT 3
#define PRIORITY_TSTOPMONITOR 3
/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_camera, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startSendImageCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopSendImageCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_finArenaCamera, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
   
    }
    if (err = rt_sem_create(&sem_findArenaCameraConfirm, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_robotStop, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_monitorStop, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_monitorConnectionLost, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
 
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraOpen, "th_cameraOpen", 0, PRIORITY_TCAMERA_ON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    } 
    
    if (err = rt_task_create(&th_cameraClose, "th_cameraClose", 0, PRIORITY_TCAMERA_OFF, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraSendImage, "th_cameraSendImage", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_cameraFindArena, "th_cameraFindArena", 0, PRIORITY_TCAMERA, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_connexionToRobotLost, "th_connexionToRobotLost", 0, PRIORITY_TCONNEXIONTOROBOTLOST, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_monitorConnexionLost, "th_monitorConnexionLost", 0, PRIORITY_TMONITORCONNEXIONLOST, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stopRobot, "th_stopRobot", 0, PRIORITY_TSTOPROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_stopMonitor, "th_stopMonitor", 0, PRIORITY_TSTOPMONITOR, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraOpen, (void(*)(void*)) & Tasks::CameraTaskOpen, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraClose, (void(*)(void*)) & Tasks::CameraTaskClose, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_cameraSendImage, (void(*)(void*)) & Tasks::CameraTaskSendImage, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
     if (err = rt_task_start(&th_cameraFindArena, (void(*)(void*)) & Tasks::CameraTaskFindArena, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_connexionToRobotLost, (void(*)(void*)) & Tasks::ConnexionToRobotLostTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_monitorConnexionLost, (void(*)(void*)) & Tasks::MonitorTaskLostConnection, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stopRobot, (void(*)(void*)) & Tasks::StopRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_stopMonitor, (void(*)(void*)) & Tasks::StopMonitorTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling getting battery status.
 */
void Tasks::BatteryTask(void *arg) {
    MessageBattery * msg;
    int rs;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    //rt_sem_p(&sem_startBattery, TM_INFINITE);
    rt_task_set_periodic(NULL, TM_NOW, 500*1000000); //<=> 0.5s
    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic battery update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if(rs==1){
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msg = (MessageBattery*)robot.Write(new Message(MESSAGE_ROBOT_BATTERY_GET)); 
            rt_mutex_release(&mutex_robot);
            WriteInQueue(&q_messageToMon, msg);
        }
    }
}
/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}


/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    int status ;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            cout << "Connection perdue avec le moniteur" << endl <<flush;
            rt_sem_v(&sem_monitorConnectionLost);  
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_sem_v(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        else if(msgRcv->CompareID(MESSAGE_CAM_OPEN)){
            rt_sem_v(&sem_startCamera)    ;
        } 
        else if(msgRcv->CompareID(MESSAGE_CAM_CLOSE)){
            rt_sem_v(&sem_stopCamera)    ;
        } 
        else if(msgRcv->CompareID(MESSAGE_CAM_ASK_ARENA)){
            rt_sem_v(&sem_finArenaCamera)    ;
        } 
        else if(msgRcv->CompareID(MESSAGE_CAM_ARENA_CONFIRM)){
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                arenaConfirmed=true;
            rt_mutex_release(&mutex_arena);
            rt_sem_v(&sem_findArenaCameraConfirm)    ;
        }else if( msgRcv->CompareID(MESSAGE_CAM_ARENA_INFIRM)){
            rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                arenaConfirmed=false;
            rt_mutex_release(&mutex_arena);
            rt_sem_v(&sem_findArenaCameraConfirm)    ;
        }
        else if( msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_START)){
            rt_mutex_acquire(&mutex_position, TM_INFINITE);
                getPosition=true;
            rt_mutex_release(&mutex_position);  
        }
        else if( msgRcv->CompareID(MESSAGE_CAM_POSITION_COMPUTE_STOP)){
            rt_mutex_acquire(&mutex_position, TM_INFINITE);
                getPosition=false;
            rt_mutex_release(&mutex_position);  
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}


/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {

        Message * msgSend;
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        cout << "Start robot without watchdog (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(robot.StartWithoutWD());
        rt_mutex_release(&mutex_robot);
        cout << msgSend->GetID();
        cout << ")" << endl;

        cout << "Movement answer: " << msgSend->ToString() << endl << flush;
        WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

        if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
        }
    }
}

/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
       // cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            robot.Write(new Message((MessageID)cpMove));
            rt_mutex_release(&mutex_robot);
        }
       // cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}
/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::CameraTaskOpen(void *arg) {
    
     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    Message *msgSend;
    bool status;
    while(1){
        rt_sem_p(&sem_startCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            status = camera.Open();
        rt_mutex_release(&mutex_camera);
        cout << "Camera open : ";
        if (status){
            cout << "success";
            msgSend = new Message(MESSAGE_ANSWER_ACK);
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                statusCameraOpenedForSending=true;
            rt_mutex_release(&mutex_camera);
            rt_sem_v(&sem_startSendImageCamera);
        }
        else {
            cout << "failed";
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                statusCameraOpenedForSending=false;
            rt_mutex_release(&mutex_camera);
        }
        WriteInQueue(&q_messageToMon, msgSend);

        cout << endl << flush;
    }
}

void Tasks::CameraTaskClose(void *arg) {
    
     cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    
    Message *msgSend;
    while(1){
        rt_sem_p(&sem_stopCamera, TM_INFINITE);
        
        
        cout << "Try closing camera" << endl << flush;
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            statusCameraOpenedForSending=false;
        rt_mutex_release(&mutex_camera);

        rt_sem_v(&sem_stopSendImageCamera);

        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            camera.Close();
        rt_mutex_release(&mutex_camera);

        cout << "Camera closed successfully" << endl << flush;
        msgSend = new Message(MESSAGE_ANSWER_ACK);
        WriteInQueue(&q_messageToMon, msgSend);     
     }
}

void Tasks::CameraTaskSendImage(void *arg) {
    MessageImg * msgSendImg;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
   
    rt_task_set_periodic(NULL, TM_NOW, 100 *1000000); 
  
    Img * image;
    Arena * arena;
    Position position;
    std::list<Position> listeRobot;
    MessagePosition * msgPosition;
    bool sCOFS,ac,gp;
    while (1) {
        rt_task_wait_period(NULL);
        rt_sem_p(&sem_startSendImageCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            sCOFS=statusCameraOpenedForSending;
        rt_mutex_release(&mutex_camera);
         while(sCOFS){
            rt_task_wait_period(NULL);
            try{
                rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                    image=new Img(camera.Grab());
                rt_mutex_release(&mutex_camera);
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    ac=arenaConfirmed;
                rt_mutex_release(&mutex_arena);
                if(!ac){
                    cout << "Arena not yet confirmed" << endl <<flush;
                    arena=new Arena(image->SearchArena());
                    if(!arena->IsEmpty()){
                        image->DrawArena(*arena);
                    }
                }else{
                    cout << "Arena yet confirmed" << endl <<flush;
                    image->DrawArena(arenaConfirmedByUser);
                    rt_mutex_acquire(&mutex_position, TM_INFINITE);
                        gp=getPosition;
                    rt_mutex_release(&mutex_position);
                     if(gp){
                         cout << "getting position" << endl <<flush;
                         listeRobot = image->SearchRobot(arenaConfirmedByUser);
                         if(!listeRobot.empty()){
                             image->DrawAllRobots(listeRobot);
                             msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, listeRobot.front());
                         }else{
                             msgPosition = new MessagePosition(MESSAGE_CAM_POSITION, position);
                         }
                         WriteInQueue(&q_messageToMon, msgPosition);
                     }
                }
                msgSendImg = new MessageImg(MESSAGE_CAM_IMAGE,image);
                WriteInQueue(&q_messageToMon, msgSendImg);
                
            }
            catch(...){
                cout << "Erreur capture"<<endl;
            }
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                sCOFS=statusCameraOpenedForSending;
                rt_mutex_release(&mutex_camera);
        } 
    }
}
        
void Tasks::CameraTaskFindArena(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    Message *msgSend;
    MessageImg * msgSendImg;
    Img * image;
    Arena * arena;
    bool ac;

    while(1){
        rt_sem_p(&sem_finArenaCamera, TM_INFINITE);
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            statusCameraOpenedForSending=false;
        rt_mutex_release(&mutex_camera);
        try{
            rt_mutex_acquire(&mutex_camera, TM_INFINITE);
                image=new Img(camera.Grab());
            rt_mutex_release(&mutex_camera);
            arena=new Arena(image->SearchArena());
            if(!arena->IsEmpty()){
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    arenaConfirmed=false;
                rt_mutex_release(&mutex_arena);

                image->DrawArena(*arena);
                
                msgSendImg = new MessageImg(MESSAGE_CAM_IMAGE,image);
                WriteInQueue(&q_messageToMon, msgSendImg); 
                        
                rt_sem_p(&sem_findArenaCameraConfirm, TM_INFINITE);
                rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                    ac = arenaConfirmed;
                rt_mutex_release(&mutex_arena);

                cout << "Arena Confirmed : ";
                if(ac){ //CONFIRM
                    cout << "yes";
                    rt_mutex_acquire(&mutex_arena, TM_INFINITE);
                        arenaConfirmedByUser = *arena;
                    rt_mutex_release(&mutex_arena);
                    msgSend = new Message(MESSAGE_ANSWER_ACK);
                }else{//INFIRM
                    cout << "no";
                    msgSend = new Message(MESSAGE_ANSWER_ACK);
                }
                cout << endl << flush;
 
            }else{
                msgSend = new Message(MESSAGE_ANSWER_NACK);
            }
            WriteInQueue(&q_messageToMon, msgSend);
        }
        catch(...){
            cout << "Erreur capture"<<endl;
            msgSend = new Message(MESSAGE_ANSWER_NACK);
            WriteInQueue(&q_messageToMon, msgSend);
        }
        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            statusCameraOpenedForSending=true;
        rt_mutex_release(&mutex_camera);
        rt_sem_v(&sem_startSendImageCamera);
     }
}
void Tasks::ConnexionToRobotLostTask(void *arg){
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task ConnexionToRobotLost starts here                                                    */
    /**************************************************************************************/
   
    int nbreEchec = 0;
    int rs;
    Message * answer;
    
    rt_task_set_periodic(NULL, TM_NOW, 2000*1000000); //2s period
    
    while(1){
        rt_task_wait_period(NULL); 
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        cout << "Test"<<endl<<flush;
        if(rs ==1){
            cout << "Connection test with robot : ";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            answer = robot.Write(new Message(MESSAGE_ROBOT_PING)); //envoie un ping au robot
            rt_mutex_release(&mutex_robot);
            
            if (!(answer->CompareID(MESSAGE_ANSWER_ACK))){
                nbreEchec++;
                cout << "failed ("<< nbreEchec << " times in a row)";
            }
            else{
                nbreEchec = 0;
                cout << "success";
            }
            cout << endl << flush;
            if(nbreEchec >= 3){
                rt_sem_v(&sem_robotStop);
            }
        }
    }
}

void Tasks::StopRobotTask(void * arg)
{
    int status;
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_robotStop, TM_INFINITE);
        cout << "Stopping robot : ";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Close();
        rt_mutex_release(&mutex_robot);
        cout << ((status <0)?"failed" : "success") << endl <<flush;
    }
}

void Tasks::StopMonitorTask(void * arg)
{
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task stopMonitor starts here                                                    */
    /**************************************************************************************/
    while (1) {;
       
        rt_sem_p(&sem_monitorStop, TM_INFINITE);
        cout << "Stopping monitor";
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Close();
        rt_mutex_release(&mutex_monitor);
        
    }
}

void Tasks::MonitorTaskLostConnection(void * arg)
{
 
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_monitorConnectionLost, TM_INFINITE);

           
        cout << "Connection Lost with Monitor"<<endl<<flush;
        rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = MESSAGE_ROBOT_STOP;
        rt_mutex_release(&mutex_move);
        
        rt_sem_v(&sem_robotStop);
        rt_sem_v(&sem_monitorStop);
        rt_sem_v(&sem_stopCamera);

        rt_mutex_acquire(&mutex_camera, TM_INFINITE);
            statusCameraOpenedForSending = false;
        rt_mutex_release(&mutex_camera);

        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 0;
        rt_mutex_release(&mutex_robotStarted);

        rt_mutex_acquire(&mutex_arena, TM_INFINITE);
            arenaConfirmed=0;
        rt_mutex_release(&mutex_arena);

        rt_mutex_acquire(&mutex_position, TM_INFINITE);
            getPosition=false;
        rt_mutex_release(&mutex_position);
        
    }
}