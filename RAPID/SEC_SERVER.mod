MODULE SEC_SERVER

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !GLOBAL VARIABLES
    !////////////////////////////////////////////////////////////////////////////////////////////////////////
    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    VAR num instructionCode;
    VAR num params{30};
    VAR num nParams;

    PERS string ipController:="192.168.125.1";
    !robot default IP
    !PERS string ipController:= "127.0.0.1"; !local IP for testing in simulation
    VAR num serverPort:=5002;

    !//Correct Instruction Execution and possible return values
    VAR num ok;
    VAR bool should_send_res;
    CONST num SERVER_BAD_MSG:=0;
    CONST num SERVER_OK:=1;

    !// Robot torque
    VAR num torque;

    !//Added by Chen Hao 2022/01/15 for (CASE 41)
    VAR num path_length :=0;

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !LOCAL METHODS
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !//Method to receive and parse the message received from a PC
    !// If correct message, loads values on:
    !// - instructionCode.
    !// - nParams: Number of received parameters.
    !// - params{nParams}: Vector of received params.
    PROC ReceiveMsg(\num wait_time)
        VAR rawbytes buffer;
        VAR num time_val := WAIT_MAX;  ! default to wait-forever
        VAR num bytes_rcvd; ! parameter received
        IF Present(wait_time) time_val := wait_time;    ! test if wait time is setted

        !//for debug
        !TPErase;
        !TPWrite "START RECEIVING";
        ClearRawBytes buffer;

        !// receive data
        SocketReceive clientSocket, \RawData:=buffer, \ReadNoOfBytes:=1024, \NoRecBytes:=bytes_rcvd, \Time:=time_val;
        !// read number of parameters
        UnpackRawBytes buffer, 1, nParams, \IntX:=UINT;
        !// read instruction code
        UnpackRawBytes buffer, 3, instructionCode, \IntX:=UINT;
        !// parameters are start from 5
        !TPWrite "param no." + NumToStr(nParams,0);
        !TPWrite "instruct no." +NumToStr(instructionCode,3);
        !TPWrite "No of bytes " + NumToStr(bytes_rcvd, 0);
        !TPWrite "No of parameters " + NumToStr((bytes_rcvd-4)/4,5);
        IF (bytes_rcvd-4)/4 <> nParams THEN
            ErrWrite \W, "Socket Recv Failed", "Did not receive expected # of bytes.",
                 \RL2:="Expected: " + ValToStr(nParams),
                 \RL3:="Received: " + ValToStr((bytes_rcvd-3)/4);
             nParams:=-1;
             RETURN;
        ELSE
            !// Read parameters (parameters are defined 4 bytes)
            IF nParams > 0 THEN
                FOR i FROM 1 TO nParams DO
                    UnpackRawBytes buffer, 5 + (i-1)*4, params{i}, \Float4;
                ENDFOR
                !TPWrite NumToStr(instructionCode,0) + " " + NumToStr(params{1},2) + " " + NumToStr(params{2},2)+ " " + NumToStr(params{3},2)+ " " + NumToStr(params{4},2)+ " " + NumToStr(params{5},2)+ " " + NumToStr(params{6},2)+ " " + NumToStr(params{7},2) +" " + NumToStr(params{8},2);
            ENDIF
        ENDIF

        ERROR
            RAISE;  ! raise errors to calling code
    ENDPROC

    !// Added by Chen Hao. This is the function to run MoveJ.
    !// If the motion exec successfully, return True
    !// If the motion exec failed, return False

    PROC ParseMsg(string msg)
        !//Local variables
        VAR bool auxOk;
        VAR num ind:=1;
        VAR num newInd;
        VAR num length;
        VAR num indParam:=1;
        VAR string subString;
        VAR bool end:=FALSE;

        !//Find the end character
        length:=StrMatch(msg,1,"#");
        IF length>StrLen(msg) THEN
            !//Corrupt message
            nParams:=-1;
        ELSE
            !//Read Instruction code
            newInd:=StrMatch(msg,ind," ")+1;
            subString:=StrPart(msg,ind,newInd-ind-1);
            auxOk:=StrToVal(subString,instructionCode);
            ! ASG: set instructionCode here!
            IF auxOk=FALSE THEN
                !//Impossible to read instruction code
                nParams:=-1;
            ELSE
                ind:=newInd;
                !//Read all instruction parameters (maximum of 8)
                WHILE end=FALSE DO
                    newInd:=StrMatch(msg,ind," ")+1;
                    IF newInd>length THEN
                        end:=TRUE;
                    ELSE
                        subString:=StrPart(msg,ind,newInd-ind-1);
                        auxOk:=StrToVal(subString,params{indParam});
                        indParam:=indParam+1;
                        ind:=newInd;
                    ENDIF
                ENDWHILE
                nParams:=indParam-1;
            ENDIF
        ENDIF
    ENDPROC

    !//Handshake between server and client:
    !// - Creates socket.
    !// - Waits for incoming TCP connection.
    PROC ServerCreateAndConnect(string ip,num port)
        VAR string clientIP;

        SocketCreate serverSocket;
        SocketBind serverSocket,ip,port;
        SocketListen serverSocket;

        !! ASG: while "current socket status of clientSocket" IS NOT EQUAL TO the "client connected to a remote host"
        WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket\ClientAddress:=clientIP\Time:=WAIT_MAX;
            !//Wait 0.5 seconds for the next reconnection
            WaitTime 0.5;
        ENDWHILE
    ENDPROC

    FUNC string FormateRes(string clientMessage)
        VAR string message;

        message:=NumToStr(instructionCode,0);
        message:=message+" "+NumToStr(ok,0);
        message:=message+" "+ clientMessage;

        RETURN message;
    ENDFUNC

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !//SERVER: Main procedure
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////

    PROC main()
        !//Local variables
        VAR string receivedString;
        !//Received string
        VAR string sendString;
        !//Reply string
        VAR string addString;
        !//String to add to the reply.
        VAR bool connected;
        !//Client connected
        VAR bool reconnected;
        !//Reconnect after sending ack
        VAR bool reconnect;
        !//Drop and reconnection happened during serving a command
        VAR jointtarget jointsPose;

        !//Socket connection
        connected:=FALSE;
        ServerCreateAndConnect ipController,serverPort;
        connected:=TRUE;
        reconnect:=FALSE;

        !//Server Loop
        WHILE TRUE DO
            !//For message sending post-movement
            should_send_res:=TRUE;
            !//Initialization of program flow variables
            ok:=SERVER_OK;
            !//Has communication dropped after receiving a command?
            addString:="";
            !//Wait for a command
            ReceiveMsg;
            !SocketReceive clientSocket\Str:=receivedString\Time:=WAIT_MAX;
            !TPWrite receivedString;
            !ParseMsg receivedString;
            !//Correctness of executed instruction.
            reconnected:=FALSE;
            !//Execution of the command
            !---------------------------------------------------------------------------------------------------------------
            TEST instructionCode
            CASE 0:
                !Ping
                IF nParams=0 THEN
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 4:
                !Get Joint Coordinates
                IF nParams=0 THEN
                    jointsPose:=CJointT();
                    addString:=NumToStr(jointsPose.robax.rax_1,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_2,2)+" ";
                    !The extra eax is essentially the third axis in our simulator
                    addString:=addString+NumToStr(jointsPose.robax.rax_3,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_4,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_5,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_6,2);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 14:
                !Get Joint Torque Currents
                IF nParams=0 THEN
                    !//addString:=NumToStr(GetMotorTorque(1),4)+" ";
                    !//addString:=addString+NumToStr(GetMotorTorque(2),4)+" ";
                    !//addString:=addString+NumToStr(GetMotorTorque(3),4)+" ";
                    !//addString:=addString+NumToStr(GetMotorTorque(4),4)+" ";
                    !//addString:=addString+NumToStr(GetMotorTorque(5),4)+" ";
                    !//addString:=addString+NumToStr(GetMotorTorque(6),4);
                    !End of string
                    GetJointData \MechUnit:=ROB_1, 1 \Torque:=torque;
                    addString:=NumToStr(torque,4)+" ";
                    GetJointData \MechUnit:=ROB_1, 2 \Torque:=torque;
                    addString:=addString+NumToStr(torque,4)+" ";
                    GetJointData \MechUnit:=ROB_1, 3 \Torque:=torque;
                    addString:=addString+NumToStr(torque,4)+" ";
                    GetJointData \MechUnit:=ROB_1, 4 \Torque:=torque;
                    addString:=addString+NumToStr(torque,4)+" ";
                    GetJointData \MechUnit:=ROB_1, 5 \Torque:=torque;
                    addString:=addString+NumToStr(torque,4)+" ";
                    GetJointData \MechUnit:=ROB_1, 6 \Torque:=torque;
                    addString:=addString+NumToStr(torque,4);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 15:
                !Get Joint Torque Currents
                IF nParams=0 THEN
                    addString:=NumToStr(GetMotorTorque(1),5)+" ";
                    addString:=addString+NumToStr(GetMotorTorque(2),5)+" ";
                    addString:=addString+NumToStr(GetMotorTorque(3),5)+" ";
                    addString:=addString+NumToStr(GetMotorTorque(4),5)+" ";
                    addString:=addString+NumToStr(GetMotorTorque(5),5)+" ";
                    addString:=addString+NumToStr(GetMotorTorque(6),5);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 98:
                !returns current robot info: serial number, robotware version, and robot type
                IF nParams=0 THEN
                    addString:=GetSysInfo(\SerialNo)+"*";
                    addString:=addString+GetSysInfo(\SWVersion)+"*";
                    addString:=addString+GetSysInfo(\RobotType);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 99:
                !Close Connection
                IF nParams=0 THEN
                    reconnect:=TRUE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            DEFAULT:
                ok:=SERVER_BAD_MSG;
            ENDTEST
            !---------------------------------------------------------------------------------------------------------------
            !Compose the acknowledge string to send back to the client
            IF connected and reconnected=FALSE and SocketGetStatus(clientSocket)=SOCKET_CONNECTED and should_send_res THEN
                IF reconnect THEN
                    connected:=FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;
                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected:=TRUE;
                    reconnected:=TRUE;
                    reconnect:=FALSE;
                ELSE
                    SocketSend clientSocket\Str:=FormateRes(addString);
                ENDIF
            ENDIF
        ENDWHILE
ERROR
        ok:=SERVER_BAD_MSG;
        should_send_res:=FALSE;

        TEST ERRNO
            CASE ERR_SOCK_CLOSED:
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                should_send_res:=TRUE;
                RETRY;

            CASE ERR_ROBLIMIT:
                ! Position is reachable but at least one axis is outside joint limit or limits exceeded for at least one coupled joint (function CalcJoinT)
                SocketSend clientSocket\Str:=FormateRes("ERR_ROBLIMIT: "+NumToStr(ERRNO,0));
                RETRY;

            CASE ERR_OUTSIDE_REACH:
                ! The position (robtarget) is outisde the robot's working area for function CalcJoinT.
                SocketSend clientSocket\Str:=FormateRes("ERR_OUTSIDE_REACH: "+NumToStr(ERRNO,0));
                RETRY;
            DEFAULT:
                SocketSend clientSocket\Str:=FormateRes("Default Error: "+NumToStr(ERRNO,0));
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                RETRY;
        ENDTEST
    ENDPROC

ENDMODULE