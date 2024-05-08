%%%
  VERSION:1
  LANGUAGE:ENGLISH
%%%

MODULE RDK_DriverSocket
    !---------------------------------------------------------------------------------------------
    ! Td
    ! Alternatively, use the RDK_DriverSerial.mod module to use an RS232 communication
    !
    ! RoboDK drivers allow you to communicate an ABB robot with RoboDK
    ! Link to RoboDK documentation to setup drivers (online programming):
    !     https://robodk.com/doc/en/Robot-Drivers.html#UseDriver
    !
    ! Link to RoboDK documentation for offline programming:
    !     https://robodk.com/doc/en/Post-Processors.html#SelectPost
    !
    ! Link to RoboDK documentation for ABB robots:
    !     https://robodk.com/doc/en/Robots-ABB.html#DriverABB
    !
    ! **** Important! ****
    !     Set the SERVER_IP variable to match the robot IP
    !
    !     Then, run the Main() or RoboDK_Driver() program from this module
    !       1- Select ABB
    !       2- Select Production Window
    !       3- Set PP to Main (Program Pointer to main). Confirm by selecting Yes
    !       4- Start the program by pressing the green "Play" button (towards the right of the teach pendant)
    !          Make sure to leave the program running
    !
    !     In RoboDK:
    !       1- Select Connect-Connect to Robot
    !       2- Make sure the driver path is "apirobot" (select More Options to see it)
    !       3- Enter the robot IP (make sure you can ping the robot)
    !       4- Select Connect
    !
    ! Update this to your robot IP ! (localhost or 127.0.0.1 may not work)
    LOCAL CONST string SERVER_IP:="192.168.125.1";

    ! Update this to an available port on the robot!
    LOCAL CONST num PORT:=2000;
    !---------------------------------------------------------------------------------------------

    ! Variables related to the socket connection
    LOCAL VAR socketdev server_socket;
    LOCAL VAR socketdev client_socket;
    VAR string client_ip;


    !--------------------------------------------------------------------------------------------
    !--------------------------------------------------------------------------------------------
    !--------------------------------------------------------------------------------------------
    !-------------- COMMON FUNCTIONS BELOW ------------------------------------------------------

    ! Global variables:
    LOCAL VAR robtarget rt_target;
    LOCAL VAR robtarget rt_target2;
    LOCAL VAR zonedata progZone;
    LOCAL VAR speeddata progSpeed;
    LOCAL PERS tooldata progTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    LOCAL PERS wobjdata progWObj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];

    ! Communication buffer:
    LOCAL VAR rawbytes bufferIn;
    LOCAL VAR rawbytes bufferOut;
    LOCAL VAR num bufferIn_Size;
    LOCAL VAR num bufferIn_Index;

    ! Main procedure
    PROC main()
        ! Reset the override speed (100% and 1000 mm/s maximum):
        VelSet 100,1000;

        ! Clear the Teach Pendant
        TPErase;

        ! Optionally set a motion supervision value or turn it off
        !MotionSup \On,\TuneValue:=100;

        ! Set smooth accelerations:
        AccSet 10,10;

        ! Set the zonedata (change to z1 for smooth movements)
        progZone:=z0;

        ! Set the default speed (SpeedData)
        progSpeed:=[500,200,0,0];

        ! Set the toolflange as the default tool:
        progTool:=tool0;

        ! Set the robot base as the default reference frame:
        progWObj:=wobj0;

        ! Tip to change the tool mass and center of gravity
        !progTool.tload.mass := 3;
        !progTool.tload.cog := [0,0,90];

        ! Avoid signularities
        ! SingArea \Wrist;

        !********** RoboDK main loop ****************
        RoboDK_Driver;
        !**********************************************

    ENDPROC



    !********** RoboDK main loop ****************
    PROC RoboDK_Driver()

        VAR num action;
        VAR num num1;
        VAR num num2;
        VAR num num3;
        VAR num num4;
        VAR jointtarget joint2go;
        VAR jointtarget joint2go2;
        VAR pos xyz;
        VAR robtarget rt_c1;
        VAR string str_value;
        VAR num number;
        VAR signaldi DI_SearchL;
        VAR signaldo DO_Signal;
        VAR signaldi DI_Signal;
        VAR dionum DIO_Value;

        ConfJ\On;
        ConfL\Off;

        COM_Start;

        TPWrite "Waiting for RoboDK Commands...";
        WHILE TRUE DO
            action:=COM_WaitCommand();
            TEST action
            CASE 0:
                ! Handshake
                number:=COM_RecvInt();
                !TPWrite "2=" \Num:=number;
                num1:=COM_RecvFloat();
                num2:=COM_RecvFloat();
                TPWrite "Connected";
                TPWrite "Driver Version: ",\num:=num1;
                !TPWrite "Version2: ", \Num:=num2;
                COM_SendInt(0);
                COM_SendFloat(3);
                COM_SendFloat(0);
            CASE 1:
                ! Number feedback
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                COM_SendInt(4);
                COM_SendFloat num1;
            CASE 2:
                ! Get Joints
                COM_SendInt(2);
                COM_SendJnts(CJointT());
            CASE 3:
                ! Move Joints
                joint2go:=COM_RecvJnts();
                COM_SendInt(1);
                ! Important: Monitor joints uses cartesian
                rt_target:=CalcRobT(joint2go,progTool);
                MoveAbsJ joint2go,progSpeed,progZone,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(127);
            CASE 23:
                ! MoveJ Using Cartesina Data
                rt_target:=COM_RecvTarget();
                COM_SendInt(1);
                MoveJ rt_target,progSpeed,progZone,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(127);
            CASE 4:
                ! Set Tool
                rt_target:=COM_RecvPose();
                progTool.tframe.trans:=rt_target.trans;
                progTool.tframe.rot:=rt_target.rot;
                COM_SendInt(127);
            CASE 5:
                ! MoveL
                joint2go:=COM_RecvJnts();
                COM_SendInt(1);
                ! Important: Monitor joints uses cartesian
                rt_target:=CalcRobT(joint2go,progTool);
                MoveL rt_target,progSpeed,progZone,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(127);
            CASE 25:
                ! MoveL Using Cartesian Data
                rt_target:=COM_RecvTarget();
                COM_SendInt(1);
                MoveL rt_target,progSpeed,progZone,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(127);
            CASE 6:
                ! SearchL
                joint2go:=COM_RecvJnts();
                COM_SendInt(1);
                ! Important: Monitor joints uses cartesian
                rt_target:=CalcRobT(joint2go,progTool);
                SearchL\PStop,DI_SearchL,rt_c1,rt_target,progSpeed,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(2);
                COM_SendJnts(CalcJointT(rt_c1,progTool,\WObj:=progWObj));
                !COM_SendJnts(CJointT());
            CASE 26:
                ! SearchL Using Cartesian Data
                rt_target:=COM_RecvTarget();
                COM_SendInt(1);
                SearchL\PStop,DI_SearchL,rt_c1,rt_target,progSpeed,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(2);
                COM_SendJnts(CalcJointT(rt_c1,progTool,\WObj:=progWObj));
                !COM_SendJnts(CJointT());
            CASE 17:
                ! MoveC
                joint2go:=COM_RecvJnts();
                joint2go2:=COM_RecvJnts();
                ! Important: Monitor joints uses cartesian
                rt_target:=CalcRobT(joint2go,progTool);
                rt_target2:=CalcRobT(joint2go2,progTool);
                COM_SendInt(1);
                MoveC rt_target,rt_target2,progSpeed,progZone,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(127);
            CASE 27:
                ! MoveC Using Cartesina Data
                rt_target:=COM_RecvTarget();
                rt_target2:=COM_RecvTarget();
                COM_SendInt(1);
                MoveC rt_target,rt_target2,progSpeed,progZone,progTool,\Wobj:=progWObj;
                RDK_MonitorMove;
                COM_SendInt(127);
            CASE 8:
                ! Set Speed
                number:=COM_RecvInt();
                ! Linear speed mm/s
                num1:=COM_RecvFloat();
                ! Joint speed deg/s
                num2:=COM_RecvFloat();
                ! Linear acceleration mm/s2
                num3:=COM_RecvFloat();
                ! Joint acceleration deg/s2
                num4:=COM_RecvFloat();
                IF num1>0 THEN
                    ! Linear speed
                    progSpeed.v_tcp:=num1;
                    progSpeed.v_leax:=num1;
                ENDIF
                IF num2>0 THEN
                    ! Orientation speed
                    progSpeed.v_ori:=num2;
                    progSpeed.v_reax:=num2;
                ENDIF
                ! WaitTime 0.01;
                COM_SendInt(127);
            CASE 9:
                ! ZoneData
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                IF num1<=0 THEN
                    progZone.finep:=TRUE;
                    num1:=0;
                ELSE
                    progZone.finep:=FALSE;
                ENDIF
                progZone.pzone_tcp:=num1;
                progZone.pzone_ori:=num1;
                progZone.pzone_eax:=num1;
                COM_SendInt(127);
            CASE 10:
                ! WaitTime
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                COM_SendInt(1);
                !WaitRob \InPos;
                IF num1>=0 THEN
                    num1:=num1/1000;
                    WaitRob\inpos;
                    TPWrite "Pause (sec): "\num:=num1;
                    WaitTime num1;
                ELSE
                    TPWrite "Program paused. Press PLAY to continue.";
                    ! Break;
                    Stop;
                ENDIF
                COM_SendInt(127);
            CASE 11:
                ! RunProgram
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                str_value:=COM_RecvStr();
                COM_SendInt(1);
                IF RDK_RunProgram(str_value) THEN
                    COM_SendInt(127);
                ELSE
                    COM_SendInt(126);
                ENDIF
            CASE 12:
                ! Pop Up
                number:=COM_RecvInt();
                str_value:=COM_RecvStr();
                COM_SendInt(1);
                WaitRob\inpos;
                TPWrite str_value;
                COM_SendInt(127);
            CASE 13:
                ! Set DO
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                num2:=COM_RecvFloat();
                !DO_Signal := num1;
                !DIO_Value := num2;
                !SetDO DO_Signal, DIO_Value;
                TPWrite "Implement setting Digital Outputs";
                Stop;
                COM_SendInt(127);
            CASE 14:
                ! Wait DI
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                num2:=COM_RecvFloat();
                !DI_Signal := num1;
                !DIO_Value := num2;
                !WaitDI DI_Signal, DIO_Value;
                TPWrite "Implement Waiting for Digital Inputs";
                Stop;
                COM_SendInt(127);
            CASE 16:
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                IF num1<=0 THEN
                    progZone.finep:=TRUE;
                    num1:=0;
                ELSE
                    progZone.finep:=FALSE;
                ENDIF
                progZone.pzone_tcp:=num1;
                progZone.pzone_ori:=num1;
                progZone.pzone_eax:=num1;
                COM_SendInt(127);
            CASE 100:
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                num2:=COM_RecvFloat();
                num3:=COM_RecvFloat();
                num4:=COM_RecvFloat();
                ! VelSet num1, num2;
                progSpeed:=[num3,num4,5000,1000];
                ! WaitTime 0.01;
                COM_SendInt(127);
!            CASE 110:
!                rt1:=COM_ReadPoseXYZRPW();
!                progTool.tframe.trans:=rt1.trans;
!                progTool.tframe.rot:=rt1.rot;
!                COM_SendInt(127);
            CASE 121:
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                num2:=COM_RecvFloat();
                !DO_Signal := num1;
                !DIO_Value := num2;
                !SetDO DO_Signal, DIO_Value;
                TPWrite "Implement setting Digital Outputs";
                COM_SendInt(127);
            CASE 122:
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                num2:=COM_RecvFloat();
                !DI_Signal := num1;
                !DIO_Value := num2;
                !WaitDI DI_Signal, DIO_Value;
                TPWrite "Implement Waiting for Digital Inputs";
                COM_SendInt(127);
            CASE 130:
                ! Added for Backwards compatibility
                number:=COM_RecvInt();
                num1:=COM_RecvFloat();
                ! Run Numbered program
                CALL_PROGRAM num1;
                COM_SendInt(127);

            DEFAULT:
                TPWrite "Unexpected command -> "\num:=action;
            ENDTEST
        ENDWHILE
        COM_End;

    ERROR
        TPWrite "Error code: "\num:=ERRNO;
        IF ERRNO=ERR_DIVZERO THEN
            TPWrite "Skipping division by zero error for small movement";
            TRYNEXT;
        ENDIF
    ENDPROC

    ! Call a program by name (it does not need to be known at compile time)
    FUNC bool RDK_RunProgram(string progname)
        WaitRob\inpos;
        TPWrite "Running program: "+progname;
        %progname %;
        WaitRob\inpos;
        TPWrite "Program "+progname+" completed";
        RETURN TRUE;
    ERROR
        TPWrite "Error running program: "+progname+". Error num: "\num:=ERRNO;
        RETURN FALSE;
    ENDFUNC

    PROC CALL_PROGRAM(num ProgId)
        ! Added for backwards compatibility
        TEST ProgId
        CASE 0:
            ! Call program 0
            ! To call this program, use one of the following options:
            !     Option 1-> Select Program-Add Program Call, then, enter the name "RunProgram 0"
            !     Option 2-> Use robot.RunInstruction('RunProgram 0', INSTRUCTION_INSERT_CODE) for online programming (using the driver)
            TPWrite "Calling program 0...";
            ! To complete
            ! .......
            Stop;
        CASE 1:
            ! Call program 1
            TPWrite "Calling program 1...";
            ! To complete
            ! .......
            Stop;
        CASE 2:
            ! Call program 2
            TPWrite "Calling program 2...";
            ! Enter a specific action or a program call
            ! To complete
            ! .......
            Stop;
        DEFAULT:
            TPWrite "Program call not implemented. ProgId = "\num:=ProgId;
        ENDTEST
    ENDPROC

    ! Monitor joint movement while the robot moves
    PROC RDK_MonitorMove()
        WHILE Distance(CPos(\Tool:=progTool,\WObj:=progWObj),rt_target.trans)>1 DO
            COM_SendInt(3);
            COM_SendJnts(CJointT());
            ! WaitTime required for RW5
            WaitTime 0.01;
        ENDWHILE
    ERROR
        IF ERRNO=ERR_WAIT_MAXTIME THEN
            ResetRetryCount;
            COM_SendInt(3);
            COM_SendJnts(CJointT());
            RETRY;
        ENDIF
    ENDPROC

    PROC COM_SendPose(robtarget paux)
        VAR num xx;
        VAR num yy;
        VAR num zz;
        VAR num rr;
        VAR num pp;
        VAR num ww;
        xx:=paux.trans.x;
        yy:=paux.trans.y;
        zz:=paux.trans.z;
        rr:=EulerZYX(\Z,paux.rot);
        pp:=EulerZYX(\Y,paux.rot);
        ww:=EulerZYX(\X,paux.rot);
        COM_SendInt(6);
        COM_SendFloat(xx);
        COM_SendFloat(yy);
        COM_SendFloat(zz);
        COM_SendFloat(rr);
        COM_SendFloat(pp);
        COM_SendFloat(ww);
    ENDPROC

    FUNC robtarget COM_RecvPose()
        VAR robtarget rtreturn:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR num xx;
        VAR num yy;
        VAR num zz;
        VAR num rr;
        VAR num pp;
        VAR num ww;
        VAR num nvalues;
        nvalues:=COM_RecvInt();
        xx:=COM_RecvFloat();
        yy:=COM_RecvFloat();
        zz:=COM_RecvFloat();
        rr:=COM_RecvFloat();
        pp:=COM_RecvFloat();
        ww:=COM_RecvFloat();
        rtreturn.trans:=[xx,yy,zz];
        rtreturn.rot:=OrientZYX(rr,pp,ww);
        RETURN rtreturn;
    ENDFUNC

    PROC COM_SendJnts(jointtarget jaux)
        ! Set to 7, 8 or more if there are more external axes
        COM_SendInt(6);
        COM_SendFloat(jaux.robax.rax_1);
        COM_SendFloat(jaux.robax.rax_2);
        COM_SendFloat(jaux.robax.rax_3);
        COM_SendFloat(jaux.robax.rax_4);
        COM_SendFloat(jaux.robax.rax_5);
        COM_SendFloat(jaux.robax.rax_6);
        ! COM_SendFloat(jreturn.extax.eax_a); ! njoints >= 7
        ! COM_SendFloat(jreturn.extax.eax_b); ! njoints >= 8
        ! ...
    ENDPROC

    FUNC jointtarget COM_RecvJnts()
        VAR jointtarget jreturn;
        VAR num naxis;
        jreturn.extax:=[9E9,9E9,9E9,9E9,9E9,9E9];
        naxis:=COM_RecvInt();
        jreturn.robax.rax_1:=COM_RecvFloat();
        jreturn.robax.rax_2:=COM_RecvFloat();
        jreturn.robax.rax_3:=COM_RecvFloat();
        jreturn.robax.rax_4:=COM_RecvFloat();
        jreturn.robax.rax_5:=COM_RecvFloat();
        jreturn.robax.rax_6:=COM_RecvFloat();
        IF naxis>=7 THEN
            jreturn.extax.eax_a:=COM_RecvFloat();
            IF naxis>=8 THEN
                jreturn.extax.eax_b:=COM_RecvFloat();
                IF naxis>=9 THEN
                    jreturn.extax.eax_c:=COM_RecvFloat();
                    IF naxis>=10 THEN
                        jreturn.extax.eax_d:=COM_RecvFloat();
                    ENDIF
                ENDIF
            ENDIF
        ENDIF
        RETURN jreturn;
    ENDFUNC

    FUNC robtarget COM_RecvTarget()
        VAR robtarget rtreturn:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR num posejoints{18}:=[0,0,0,0,0,0,9E+09,9E+09,9E+09,9E+09,9E+09,9E+09,0,0,0,0,0,0];
        VAR num nvals;
        ! var num naxes;
        VAR num xx;
        VAR num yy;
        VAR num zz;
        VAR num rr;
        VAR num pp;
        VAR num ww;
        ! Always 6 or more
        nvals:=COM_RecvInt();
        FOR i FROM 1 TO nvals DO
            posejoints{i}:=COM_RecvFloat();
        ENDFOR
        ! naxes := nvals - 6;
        xx:=posejoints{nvals-5};
        yy:=posejoints{nvals-4};
        zz:=posejoints{nvals-3};
        rr:=posejoints{nvals-2};
        pp:=posejoints{nvals-1};
        ww:=posejoints{nvals-0};
        rtreturn.trans:=[xx,yy,zz];
        rtreturn.rot:=OrientZYX(rr,pp,ww);
        ! Get joints info (joints 1-6 are ignored)
        rtreturn.extax.eax_a:=posejoints{7};
        rtreturn.extax.eax_b:=posejoints{8};
        rtreturn.extax.eax_c:=posejoints{9};
        rtreturn.extax.eax_d:=posejoints{10};
        rtreturn.extax.eax_e:=posejoints{11};
        rtreturn.extax.eax_f:=posejoints{12};
        RETURN rtreturn;
    ENDFUNC

    PROC COM_Debug()
        VAR num num_send;
        VAR num num_recv;
        WaitTime 5;
        num_send:=1;
        WHILE TRUE DO
            !num_recv := ReadBin(RDK_COM);
            !TPWrite "Recv number: " \Num:=num_recv;
            TPWrite "Sending float: "\num:=num_send;
            !COM_SendFloat(num_send);
            COM_SendInt(num_send);
            WaitTime 1;
            num_send:=num_send+1;
        ENDWHILE
    ENDPROC

    ! Common receive functions (compatible with Serial and Socket)
    FUNC num COM_RecvInt()
        VAR num value;
        UnpackRawBytes bufferIn,bufferIn_Index,value\IntX:=DINT;
        bufferIn_Index:=bufferIn_Index+4;
        RETURN value;
    ENDFUNC

    FUNC num COM_RecvFloat()
        VAR num value;
        UnpackRawBytes bufferIn,bufferIn_Index,value\Float4;
        bufferIn_Index:=bufferIn_Index+4;
        RETURN value;
    ENDFUNC

    FUNC string COM_RecvStr()
        VAR string str_array;
        VAR num array_sz;
        array_sz:=COM_RecvInt();
        UnpackRawBytes bufferIn,bufferIn_Index,str_array\ASCII:=array_sz;
        bufferIn_Index:=bufferIn_Index+array_sz;
        RETURN str_array;
    ENDFUNC

    !-------------- COMMON FUNCTIONS ABOVE -------------------


    !-------------------------------------------------------------
    !-------------------------------------------------------------
    !-------------------------------------------------------------
    ! Low level SOCKET communication functions
    PROC COM_Start()
        COM_Server_Recover;
        !Debug_Option;
    ENDPROC

    PROC COM_End()
        SocketClose server_socket;
        SocketClose client_socket;
        WaitTime 2;
    ENDPROC

    FUNC num COM_WaitCommand()
        VAR num action;
        VAR byte onebyte;
        VAR rawbytes chunkbytes;
        VAR num buffer_len;

        ! Wait for a new command
        SocketReceive client_socket,\RawData:=bufferIn,\ReadNoOfBytes:=1,\Time:=WAIT_MAX;
        bufferIn_Index:=1;
        UnpackRawBytes bufferIn,bufferIn_Index,bufferIn_Size,\IntX:=USINT;

        ! Receive byte by byte
        ClearRawBytes bufferIn;
        SocketReceive client_socket,\RawData:=bufferIn,\ReadNoOfBytes:=bufferIn_Size,\Time:=1;

        ! Retrieve the command (first 4 bytes as int)
        bufferIn_Index:=1;
        action:=COM_RecvInt();
        RETURN action;
    ERROR
        IF ERRNO=ERR_SOCK_CLOSED THEN
            COM_Server_Recover;
            RETRY;
        ENDIF
    ENDFUNC

    PROC COM_Server_Recover()
        CONST num VERSION:=1;
        CONST num TypeAngles:=1;
        CONST num ROBOT_ID:=-1;
        VAR num aux;
        TPErase;
        SocketClose server_socket;
        SocketClose client_socket;
        SocketCreate server_socket;
        SocketBind server_socket,SERVER_IP,PORT;
        SocketListen server_socket;
        TPWrite "Waiting for connection...";
        SocketAccept server_socket,client_socket\ClientAddress:=client_ip,\Time:=WAIT_MAX;
        TPWrite "Connected to client:";
        TPWrite "Listening to TCP/IP commands...";
    ERROR
        IF ERRNO=ERR_SOCK_TIMEOUT THEN
            RETRY;
        ELSEIF ERRNO=ERR_SOCK_CLOSED THEN
            RETURN ;
        ELSE
            ! No error recovery handling
        ENDIF
    ENDPROC

    PROC COM_SendInt(num value)
        PackRawBytes value,bufferOut,1\IntX:=DINT;
        SocketSend client_socket,\RawData:=bufferOut,\NoOfBytes:=4;
    ENDPROC

    PROC COM_SendFloat(num value)
        PackRawBytes value,bufferOut,1\Float4;
        SocketSend client_socket,\RawData:=bufferOut,\NoOfBytes:=4;
    ENDPROC

ENDMODULE
