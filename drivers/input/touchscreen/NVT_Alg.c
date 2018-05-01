//==============================================================//
//			Include Files				//
//==============================================================//
#include "NVT_Alg.h"
#include <linux/input.h>
//#include <linux/input/NVTtouch.h>

//==============================================================//
//		 	Define Const Values			//
//==============================================================//
//#define abs(a) ((a<0)?-a:a)
#define STATUS_NONE 	0
#define STATUS_IN   	1
#define STATUS_MOVE  	2
#define STATUS_BREAK  	3

#define DEBUG_MODE      0

#if DEBUG_MODE
#define NVT_ALG_INFO(format, arg...)	\
	printk(KERN_INFO "nvt: [%s] " format , __FUNCTION__ , ## arg)
#else
#define NVT_ALG_INFO(format, arg...)
#endif
#define NVT_ALG_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "nvt: [%s] " format , __FUNCTION__ , ## arg)

#define NVT_ALG_ERR(format, arg...)	\
	printk(KERN_ERR "nvt: [%s] " format , __FUNCTION__ , ## arg)

unsigned char FW_EnterDebounce[20] = {Drp_Frm_NC_ID1,Drp_Frm_NC_ID2,Drp_Frm_NC_ID3,Drp_Frm_NC_ID4,Drp_Frm_NC_ID5,Drp_Frm_NC_ID6,Drp_Frm_NC_ID7,Drp_Frm_NC_ID8,Drp_Frm_NC_ID9,Drp_Frm_NC_ID10,Drp_Frm_NC_ID11,Drp_Frm_NC_ID12,Drp_Frm_NC_ID13,Drp_Frm_NC_ID14,Drp_Frm_NC_ID15,Drp_Frm_NC_ID16,Drp_Frm_NC_ID17,Drp_Frm_NC_ID18,Drp_Frm_NC_ID19,Drp_Frm_NC_ID20};

//==============================================================//
//			Declaration Function			//
//==============================================================//
void TrackingFunc_MinDis(int TrackThreshold, unsigned char X_Jitter_Max, unsigned char Y_Jitter_Max, unsigned char X_Jitter_Offset, unsigned char Y_Jitter_Offset, int IIR_Weight, int IIR_Start, unsigned int IIR_Distance);

void Boundary(int Max_X, int Max_Y, int XEdgeDist_H, int XEdgeDist_L, int YEdgeDist_H, int YEdgeDist_L, unsigned char XCorrectionGradient_H, unsigned char XCorrectionGradient_L, unsigned char YCorrectionGradient_H, unsigned char YCorrectionGradient_L);

unsigned long int Distance_Sqrt(unsigned long int PosX1, unsigned long int PosY1, unsigned long int PosX2, unsigned long int PosY2);

static unsigned int getReportPoint(char option, char ID);
static unsigned int getRawPoint(char option, char ID);

static unsigned int getPreRawPoint(char option, char ID);

static unsigned int getPreIndex(char ID);

void Report_Buffer_Before(int);

void Report_Buffer_After(int bufferNum);

void Para_Parser(unsigned char *pData);

static void Point_FIR(unsigned char index, char MIN_Fir_Cnt, char MAX_Fir_Cnt,unsigned int Fir_Step);

unsigned int getArea(unsigned char index);

unsigned int getPressure(unsigned char index);

static void MedianFilter_init(unsigned char ID, unsigned int crood_x, unsigned int crood_y);

static void MedianFilter(unsigned short InX, unsigned short InY, unsigned char ID, unsigned char Order, unsigned char index);

static signed int getCrossValue(int pos1_x, int pos1_y,int pos2_x, int pos2_y, int testPos_x, int testPos_y);

static void crossCheckHandler(void);

static void smoothing(unsigned char X_Jitter_Max, unsigned char Y_Jitter_Max, unsigned char X_Jitter_Offset, unsigned char Y_Jitter_Offset, int IIR_Weight, int IIR_Start, unsigned int IIR_Distance);

//==============================================================//
//		 Extern Gloubal Variables			//
//==============================================================//





/****************************************************************

	      	     Distance Function

****************************************************************/
unsigned long int Distance(unsigned long int PosX1, unsigned long int PosY1, unsigned long int PosX2, unsigned long int PosY2, unsigned int Distance)
{
    if((unsigned long int)((PosX1-PosX2)*(PosX1-PosX2)+(PosY1-PosY2)*(PosY1-PosY2))>Distance)
	    return 0xFFFFFFF;
    else
	    return (unsigned long int)((PosX1-PosX2)*(PosX1-PosX2)+(PosY1-PosY2)*(PosY1-PosY2));
}


/****************************************************************

	      	     Distance Square Root Function

****************************************************************/
unsigned long int Distance_Sqrt(unsigned long int PosX1, unsigned long int PosY1, unsigned long int PosX2, unsigned long int PosY2)
{
    unsigned long int Dis_X, Dis_Y;

    if(PosX1 > PosX2)
    {
        var_temp1 = DIR_NEG;
        Dis_X = (PosX1 - PosX2);
    }
    else
    {
        var_temp1 = DIR_POS;
        Dis_X = (PosX2 - PosX1);
    }

    if(PosY1 > PosY2)
    {
        var_temp2 = DIR_NEG;
        Dis_Y = (PosY1 - PosY2);
    }
    else
    {
        var_temp2 = DIR_POS;
        Dis_Y = (PosY2 - PosY1);
    }

    var_temp3 = Dis_X;
    var_temp4 = Dis_Y;

    return (unsigned long int)(Dis_X + Dis_Y);
}



/***************************************************************

		    Tracking Algorithm

***************************************************************/
void TrackingFunc_MinDis(int TrackThreshold, unsigned char X_Jitter_Max, unsigned char Y_Jitter_Max, unsigned char X_Jitter_Offset, unsigned char Y_Jitter_Offset, int IIR_Weight, int IIR_Start, unsigned int IIR_Distance)
{
    int count, i, j, k, ID_CNT=0;
    unsigned long int min_Dis;
	unsigned char tmp_i = 0;
	unsigned char tmp_j = 0;
    unsigned int Dis_XY;
    //unsigned long int Dis_Table[10][10];
    unsigned long int Dis_Table[20][10];
    //unsigned char Drop_Frm_AC[20] = {Drp_Frm_AC_ID1,Drp_Frm_AC_ID2,Drp_Frm_AC_ID3,Drp_Frm_AC_ID4,Drp_Frm_AC_ID5,Drp_Frm_AC_ID6,Drp_Frm_AC_ID7,Drp_Frm_AC_ID8,Drp_Frm_AC_ID9,Drp_Frm_AC_ID10,Drp_Frm_AC_ID11,Drp_Frm_AC_ID12,Drp_Frm_AC_ID13,Drp_Frm_AC_ID14,Drp_Frm_AC_ID15,Drp_Frm_AC_ID16,Drp_Frm_AC_ID17,Drp_Frm_AC_ID18,Drp_Frm_AC_ID19,Drp_Frm_AC_ID20};
	//unsigned char Drop_Frm_DC[20] = {Drp_Frm_DC_ID1,Drp_Frm_DC_ID2,Drp_Frm_DC_ID3,Drp_Frm_DC_ID4,Drp_Frm_DC_ID5,Drp_Frm_DC_ID6,Drp_Frm_DC_ID7,Drp_Frm_DC_ID8,Drp_Frm_DC_ID9,Drp_Frm_DC_ID10,Drp_Frm_DC_ID11,Drp_Frm_DC_ID12,Drp_Frm_DC_ID13,Drp_Frm_DC_ID14,Drp_Frm_DC_ID15,Drp_Frm_DC_ID16,Drp_Frm_DC_ID17,Drp_Frm_DC_ID18,Drp_Frm_DC_ID19,Drp_Frm_DC_ID20};
	//unsigned char Drop_Frm_NC[20] = {Drp_Frm_NC_ID1,Drp_Frm_NC_ID2,Drp_Frm_NC_ID3,Drp_Frm_NC_ID4,Drp_Frm_NC_ID5,Drp_Frm_NC_ID6,Drp_Frm_NC_ID7,Drp_Frm_NC_ID8,Drp_Frm_NC_ID9,Drp_Frm_NC_ID10,Drp_Frm_NC_ID11,Drp_Frm_NC_ID12,Drp_Frm_NC_ID13,Drp_Frm_NC_ID14,Drp_Frm_NC_ID15,Drp_Frm_NC_ID16,Drp_Frm_NC_ID17,Drp_Frm_NC_ID18,Drp_Frm_NC_ID19,Drp_Frm_NC_ID20};
    //unsigned char TrackNum = 0;
    unsigned char TrackTimes;
    unsigned char trackMask[20] = {0};
    unsigned char trackIndex = 0;
    unsigned char Tapping_Check[20] = {0};
    unsigned char trackedID;

    //NVT_ALG_NOTICE("Tracking \n");

    //Distance
    for (i=0;i<20;i++)
    {
	    for (j=0;j<10;j++)
	    {
		    Dis_Table[i][j]=0xFFFFFFF;
	    }
    }

	for (i=0; i<prePointState.points; i++)
    {
        for (j=0; j<mPointState.points; j++)
	    {
		    Dis_Table[i][j] = Distance(prePointState.PosX[i], prePointState.PosY[i], mPointState.PosX[j], mPointState.PosY[j], TrackThreshold);
	    }
    }

    for (i=0; i<MAXID; i++)
    {
        if ( (ID_Valid[i] == 0) && (ID_Table[i] == 1) )
            trackMask[i] = 1;
        else
            trackMask[i] = 0;
    }

    TrackTimes = mPointState.points;

    maxTrackDis = 0;
    trackedNum = 0;
    //priority tracking start
    for (trackIndex = 0; trackIndex < TRACKLEVEL; trackIndex++)
    {
        if (trackIndex == 1)
        {
            if (trackedNum < mPointState.points)
                TrackTimes = (mPointState.points - trackedNum);
            else
                break;
        }

        for (count=0; count < TrackTimes; count++)
        {
            min_Dis = 0xFFFFFFF;
            for (i=0; i<prePointState.points; i++)
            {
                if ( (trackIndex==0) && (trackMask[prePointState.ID[i]]==1) )
                    continue;

    	        for(j=0; j<mPointState.points; j++)
    	        {
    	    	    if(Dis_Table[i][j] < min_Dis)
    	    	    {
    		            min_Dis = Dis_Table[i][j];
    		            tmp_i = i;
    		            tmp_j = j;
    		        }
    	        }
            }

            if (min_Dis != 0xFFFFFFF)
            {

                Dis_XY = Distance_Sqrt(prePointState.PosX[tmp_i], prePointState.PosY[tmp_i], mPointState.PosX[tmp_j], mPointState.PosY[tmp_j]);

                if (Dis_XY > maxTrackDis)
    			    maxTrackDis = Dis_XY;

                if (Dis_XY <= (FW_TrackThreshold_H*256 + FW_TrackThreshold_L))
                {

                    trackedID = prePointState.ID[tmp_i];
                    trackedIDList[trackedNum] = trackedID;
                    trackedNum++;

                #if G_FINGER_EN
                    if ( (direction_X[trackedID] == var_temp1) && (var_temp3 > FW_Boundary_DisX_TH) && (var_temp4 < FW_Boundary_DY_TH) )
                    {
						//NVT_ALG_NOTICE("G Finger~~ %d %d %d\n", trackedID, sameDirCnt_X[trackedID], lastVector_X[trackedID]);

                        sameDirCnt_X[trackedID]++;
                        if (sameDirCnt_X[trackedID] > FW_Same_Dir_Cnt_X)
                            sameDirCnt_X[trackedID] = FW_Same_Dir_Cnt_X;
                    }
                    else
                    {
                        direction_X[trackedID] = var_temp1;
                        sameDirCnt_X[trackedID] = 0;
                    }
                    //lastVector_X[trackedID] = mPointState.PosX[tmp_j] - prePointState.PosX[tmp_i];

                    if ( (direction_Y[trackedID] == var_temp2) && (var_temp4 > BOUNDARY_CHEAT_DIS_Y_TH) && (var_temp3 < BOUNDARY_CHEAT_DX_TH) )
                    {
                        sameDirCnt_Y[trackedID]++;
                        if (sameDirCnt_Y[trackedID] > SAME_DIR_CNT_Y_TH)
                            sameDirCnt_Y[trackedID] = SAME_DIR_CNT_Y_TH;
                    }
                    else
                    {
                        direction_Y[trackedID] = var_temp2;
                        sameDirCnt_Y[trackedID] = 0;
                    }
                    //lastVector_Y[trackedID] = mPointState.PosY[tmp_j] - prePointState.PosY[tmp_i];
                #endif


                    mPointState.Status[tmp_j] = STATUS_MOVE;
                    prePointState.Status[tmp_i] = STATUS_IN;
                    mPointState.ID[tmp_j] = trackedID;

                    //Finger debounce
		            Enter_Debounce[trackedID]++;
                    if(Enter_Debounce[trackedID] > MAX_ENTER_DB)
                    {
                        Enter_Debounce[trackedID] = MAX_ENTER_DB;
                    }

                    Break_Debounce[trackedID] = 0;

                    //Tapping Check Start
                    if (Dis_XY == 0)
                        Dis_XY = 1;  //for Tapping

                    if ( (Large_Move_Cnt[mPointState.ID[tmp_j]] < LARGE_MOVE_CNT_TH) && (Dis_XY > (FW_Tapping_Dis_H*256 + FW_Tapping_Dis_L)) && ((Dis_XY/Velocity[mPointState.ID[tmp_j]]) > FW_Tapping_Ratio) )
					{
						Tapping_Check[trackedID] = 1;

						if (Large_Move_Cnt[trackedID] > 0)
							Large_Move_Cnt[trackedID]--;
					}
                    else
                        Tapping_Check[trackedID] = 0;


                    Velocity[trackedID] = Dis_XY;

                    if (Dis_XY > TAPPING_LARGE_MOVE_TH)     //added by TY Chan Test 2013.07.23
                    {
                        Large_Move_Cnt[trackedID]++;

                        if (Large_Move_Cnt[trackedID] >= LARGE_MOVE_MAX)
                            Large_Move_Cnt[trackedID] = LARGE_MOVE_MAX;
                    }
                    else
                        Large_Move_Cnt[trackedID] = 0;

                    //Tapping Check End

                    for (k=0; k<prePointState.points; k++)
        	        {
        		         Dis_Table[k][tmp_j] = 0xFFFFFFF;
                    }

                    for (k=0; k<mPointState.points; k++)
        	        {
        		         Dis_Table[tmp_i][k] = 0xFFFFFFF;
        	        }
                }
            }
        }
    }
    //priority tracking end

    //Cross check
    if (FW_Tracking_Anti_Cross_En == 1)
    {
        crossCheckHandler();
    }

    //smoothing Start
    smoothing(X_Jitter_Max, Y_Jitter_Max, X_Jitter_Offset, Y_Jitter_Offset, IIR_Weight, IIR_Start, IIR_Distance);

    //New Point with Enter ID Debounce
    for (i=0; i<mPointState.points; i++)
    {
	    if (mPointState.Status[i] == STATUS_IN)
	    {
    		for (ID_CNT=0; ID_CNT<MAXID; ID_CNT++)
    		{
    			if ((ID_Table[ID_CNT] == 0) && (ReUseIDCnt[ID_CNT] >= ID_REUSE_CNT))
    				break;
    		}

		    if (ID_CNT == MAXID || trackedNum >= MAXID)
			    break;

            Velocity[ID_CNT] = 1;
            mPointState.ID[i] = ID_CNT;

            if (FW_FIR_EN == 1)
                Point_FIR(i, FW_Min_Fir_Cnt, FW_Max_Fir_Cnt, (FW_Fir_Step_H*256 + FW_Fir_Step_L));

            mPointState.Status[i] = STATUS_MOVE;
	        ID_Table[ID_CNT] = 1;
            Enter_Debounce[ID_CNT] = 1;
            Break_Debounce[ID_CNT] = 0;
		    IIRPointState.PosX[i] = mPointState.PosX[i];
        	IIRPointState.PosY[i] = mPointState.PosY[i];
            ReportPointState.PosX[i] = mPointState.PosX[i];
     	    ReportPointState.PosY[i] = mPointState.PosY[i];
            ID_Valid[ID_CNT] = 0;

            MedianFilter_init(ID_CNT, ReportPointState.PosX[i], ReportPointState.PosY[i]);

            /*
			if (FW_PWR_State == AC_MODE)
            	FinDown_TH[ID_CNT] = Drop_Frm_AC[trackedNum];
			else if (FW_PWR_State == DC_MODE)
				FinDown_TH[ID_CNT] = Drop_Frm_DC[trackedNum];
			else if (FW_PWR_State == NC_MODE)
				FinDown_TH[ID_CNT] = Drop_Frm_NC[trackedNum];
			else
				FinDown_TH[ID_CNT] = Drop_Frm_AC[trackedNum];
            */

            FinDown_TH[ID_CNT] = FW_EnterDebounce[trackedNum];

            Large_Move_Cnt[ID_CNT] = 0;
	    }
    }

    //Enter ID Debounce
    for (i=0; i<MAXID; i++)
    {
        if ((ID_Valid[i] == 0) && (ID_Table[i] == 1) && (Enter_Debounce[i] >= FinDown_TH[i]))
        {

        #if DEBUG_MODE
	        if (ID_Valid[i] == 0)
	    	    NVT_ALG_NOTICE("Enter ID = %d\n", i);
        #endif

            ID_Valid[i] = 1;
	        FinDown_TH[i] = FIN_DOWN_TH;
        }

        //Tapping Check
        if (Tapping_Check[i] == 1)
        {
			//NVT_ALG_NOTICE("Tapping ID = %d\n", i);

            ID_Valid[i] = 0;

            if (FW_FIR_EN == 1)
            {
                for (j=0; j<10; j++)
                {
                    FIR_X[i][j] = getRawPoint(OPTION_X, i);
                    FIR_Y[i][j] = getRawPoint(OPTION_Y, i);
                }
            }

			MedianFilter_init(i, (getRawPoint(OPTION_X, i)), (getRawPoint(OPTION_Y, i)));
        }
    }

    //Break
    for (k=0; k<prePointState.points; k++)
    {
	    if (prePointState.Status[k]==STATUS_MOVE)
	    {
            Break_Debounce[prePointState.ID[k]]++;

            if ( (ID_Valid[prePointState.ID[k]] == 0) || (Break_Debounce[prePointState.ID[k]] >= FW_FinUP_TH) )
            {

			#if DEBUG_MODE
        		if(ID_Valid[prePointState.ID[k]] == 1)
        		{
				NVT_ALG_NOTICE("Break ID = %d\n", prePointState.ID[k]);
        		}
			#endif

                Break_Debounce[prePointState.ID[k]] = 0;
                prePointState.Status[k] = STATUS_BREAK;
                ID_Table[prePointState.ID[k]] = 0;
                Enter_Debounce[prePointState.ID[k]] = 0;
                ID_Valid[prePointState.ID[k]] = 0;
            }
            else if ((ID_Valid[prePointState.ID[k]] == 1) && (Break_Debounce[prePointState.ID[k]] < FW_FinUP_TH))
            {
        		if (mPointState.points < 20)
        		{
                    //NVT_ALG_NOTICE("Break~~ %d %d %d\n", prePointState.ID[k], sameDirCnt_X[prePointState.ID[k]], preReportPointState.PosX[k]);

                #if G_FINGER_EN
                    if ( (sameDirCnt_X[prePointState.ID[k]] == FW_Same_Dir_Cnt_X) && ((preReportPointState.PosX[k] < (FW_Boundary_Range_H*256 + FW_Boundary_Range_L)) || (preReportPointState.PosX[k] > (C_X_Resolution - (FW_Boundary_Range_H*256 + FW_Boundary_Range_L)))) )
                    {
                        preReportPointState.PosX[k] += lastVector_X[prePointState.ID[k]];
                        preReportPointState.PosY[k] += lastVector_Y[prePointState.ID[k]];
						direction_X[prePointState.ID[k]] = 0;
                        direction_Y[prePointState.ID[k]] = 0;
                        sameDirCnt_X[prePointState.ID[k]] = 0;
                        sameDirCnt_Y[prePointState.ID[k]] = 0;
						Break_Debounce[prePointState.ID[k]]--;

						//NVT_ALG_NOTICE("G Finger Hit~~ %d\n", lastVector_X[prePointState.ID[k]]);
                    }
                    else
                    {
                        direction_X[prePointState.ID[k]] = 0;
                        direction_Y[prePointState.ID[k]] = 0;
                        sameDirCnt_X[prePointState.ID[k]] = 0;
                        sameDirCnt_Y[prePointState.ID[k]] = 0;
                    }
                #endif

        	        ReportPointState.PosX[mPointState.points] = preReportPointState.PosX[k];
        			ReportPointState.PosY[mPointState.points] = preReportPointState.PosY[k];
        	        IIRPointState.PosX[mPointState.points] = preIIRPointState.PosX[k];
        	        IIRPointState.PosY[mPointState.points] = preIIRPointState.PosY[k];
        	        mPointState.PosX[mPointState.points] = prePointState.PosX[k];
        	        mPointState.PosY[mPointState.points] = prePointState.PosY[k];
        	        mPointState.Status[mPointState.points] = prePointState.Status[k];
        	        mPointState.ID[mPointState.points] = prePointState.ID[k];
        	        mPointState.points++;
        		}
        		else
        		{
					//NVT_ALG_NOTICE("ID reah max!!!\n");

        			Break_Debounce[prePointState.ID[k]] = 0;
        		    prePointState.Status[k] = STATUS_BREAK;
        		    ID_Table[prePointState.ID[k]] = 0;
        		    Enter_Debounce[prePointState.ID[k]] = 0;
        		    ID_Valid[prePointState.ID[k]] = 0;
		        }
            }
            Large_Move_Cnt[prePointState.ID[k]] = 0;
	    }
    }

	//ID re-use debounce
	for (k=0; k<MAXID; k++)
	{
		if (ID_Table[k] == 0)
		{
			ReUseIDCnt[k]++;
			if (ReUseIDCnt[k] > ID_REUSE_CNT)
				ReUseIDCnt[k] = ID_REUSE_CNT;
		}
		else
		{
			ReUseIDCnt[k] = 0;
		}
	}

}



/*******************************************************

	          Boundary Algorithm

*******************************************************/
void Boundary(int Max_X, int Max_Y, int XEdgeDist_H, int XEdgeDist_L, int YEdgeDist_H, int YEdgeDist_L, unsigned char XCorrectionGradient_H, unsigned char XCorrectionGradient_L, unsigned char YCorrectionGradient_H, unsigned char YCorrectionGradient_L)
{
    int i;
    for(i=0;i<10;i++)
    {
		if(ReportPointState.PosX[i]<XEdgeDist_L)
		{
			Boundary_ReportPointState.PosX[i]=ReportPointState.PosX[i] - (((XEdgeDist_L - ReportPointState.PosX[i])*XCorrectionGradient_L)/16);
			if(Boundary_ReportPointState.PosX[i]<0)
			    Boundary_ReportPointState.PosX[i]=0;
		}
		else if(ReportPointState.PosX[i]>(Max_X - XEdgeDist_H))
		{
			Boundary_ReportPointState.PosX[i]=ReportPointState.PosX[i] + (((ReportPointState.PosX[i] - (Max_X - XEdgeDist_H))*XCorrectionGradient_H)/16);
			if(Boundary_ReportPointState.PosX[i]>Max_X)
			    Boundary_ReportPointState.PosX[i]=Max_X;
		}
		else
			Boundary_ReportPointState.PosX[i] = ReportPointState.PosX[i];

		if(ReportPointState.PosY[i]<YEdgeDist_L)
		{
			Boundary_ReportPointState.PosY[i]=ReportPointState.PosY[i] - (((YEdgeDist_L - ReportPointState.PosY[i])*YCorrectionGradient_L)/16);
			if(Boundary_ReportPointState.PosY[i]<0)
			    Boundary_ReportPointState.PosY[i]=0;
		}
		else if(ReportPointState.PosY[i]>(Max_Y - YEdgeDist_H))
		{
			Boundary_ReportPointState.PosY[i]=ReportPointState.PosY[i] + (((ReportPointState.PosY[i] - (Max_Y - YEdgeDist_H))*YCorrectionGradient_H)/16);
			if(Boundary_ReportPointState.PosY[i]>Max_Y)
			    Boundary_ReportPointState.PosY[i]=Max_Y;
		}
		else
			Boundary_ReportPointState.PosY[i] = ReportPointState.PosY[i];

    }
}


//Get Boundary_ReportPoint of ID~
static unsigned int getReportPoint(char option, char ID)
{
    int i;
    unsigned int point;

    for (i=0; i<mPointState.points; i++)
    {
        if (mPointState.ID[i] == ID)
            break;
    }

    if (i == mPointState.points)
        point = (0xFFFF);
    else if (option == OPTION_X)
    {
        point = Boundary_ReportPointState.PosX[i];
    }
    else if (option == OPTION_Y)
    {
        point = Boundary_ReportPointState.PosY[i];
    }
    else
        point = (0xFFFF);

    return point;
}
//Get raw coordinate of ID~
static unsigned int getRawPoint(char option, char ID)
{
    int i;
    unsigned int point;

    for (i=0; i<mPointState.points; i++)
    {
        if (mPointState.ID[i] == ID)
            break;
    }

    if (i == mPointState.points)
        point = (0xFFFF);
    else if (option == OPTION_X)
    {
        point = mPointState.PosX[i];
    }
    else if (option == OPTION_Y)
    {
        point = mPointState.PosY[i];
    }
    else
        point = (0xFFFF);

    return point;

}


//Get pre-raw coordinate of ID~
static unsigned int getPreRawPoint(char option, char ID)
{
    int i;
    unsigned int point;

    for (i=0; i<prePointState.points; i++)
    {
        if (prePointState.ID[i] == ID)
            break;
    }

    if (i == prePointState.points)
	{
        point = (0xFFFF);
	}
	else if (option == OPTION_X)
    {
        point = prePointState.PosX[i];
    }
    else if (option == OPTION_Y)
    {
        point = prePointState.PosY[i];
    }
    else
        point = (0xFFFF);

    return point;
}


static unsigned int getPreIndex(char ID)
{
    unsigned int i;

    for (i=0; i<prePointState.points; i++)
    {
        if (prePointState.ID[i] == ID)
            break;
    }

    return i;
}


//added by TY Chan 2013.07.29
void Report_Buffer_Before(int bufferNum)
{
    int i, j, ID;

	if (bufferNum==0)
		return;


	for (i=0; i<mPointState.points; i++)
	{
		ID = mPointState.ID[i];

		if ( (ID_Valid_pre[ID] == 0) && (ID_Valid[ID] == 1) ) //first report case
        {
            for (j=0; j<bufferNum; j++)
            {
                Report_Buffer_X[ID][j] = getReportPoint(OPTION_X, ID);   //get x coordinate
                Report_Buffer_Y[ID][j] = getReportPoint(OPTION_Y, ID);   //get y coordinate
            }
        }
        else if ( (ID_Valid_pre[ID] == 1) && (ID_Valid[ID] == 1) )    //move case
        {
            if (FinUP_Update == 1)
            {
                for (j=0; j<bufferNum; j++)
                {
                    Report_Buffer_X[ID][j] = getReportPoint(OPTION_X, ID);   //get x coordinate
                    Report_Buffer_Y[ID][j] = getReportPoint(OPTION_Y, ID);   //get y coordinate
                }
                FinUP_Update = 0;
            }
            else
            {
                Report_Buffer_X[ID][bufferNum-1] = getReportPoint(OPTION_X, ID);
                Report_Buffer_Y[ID][bufferNum-1] = getReportPoint(OPTION_Y, ID);
            }
        }
        else if ( (ID_Valid_pre[ID] == 1) && (ID_Valid[ID] == 0) )    //break case
        {
            return;
        }
	}

    return;
}


void Report_Buffer_After(int bufferNum)
{
    int i, j, ID;

	if (bufferNum<=1)
		return;

	for (i=0; i<mPointState.points; i++)
    {
		ID = mPointState.ID[i];

        if (ID_Valid[ID] == 1)
        {
            for (j=0; j<(bufferNum-1); j++) //shift buffer
            {
                Report_Buffer_X[ID][j] = Report_Buffer_X[ID][j+1];
                Report_Buffer_Y[ID][j] = Report_Buffer_Y[ID][j+1];
            }
        }
    }

	return;
}

static unsigned int two_power(unsigned int num)
{
	int i;
	unsigned int value = 2;

	if (num == 0)
		return 1;

	for (i=0; i<num; i++)
	{
		if (i!=0)
		{
			value = value*2;
		}
	}

	return value;
}

//Parse parameter from FW
void Para_Parser(unsigned char *pData)
{
    static unsigned char init = 0;
    static unsigned char FW_Update = 0;

	//default setting
    if (init == 0)
    {
		init = 1;

		FW_IIR_Distance_L = C_IIR_Distance_L;
        FW_IIR_Distance_H = C_IIR_Distance_H;
		FW_IIR_Weight = C_IIR_Weight;
		FW_IIR_Start = C_IIR_Start;
		FW_FinUP_TH = FIN_UP_TH;
		FW_TrackThreshold_H = TRACK_TH_H;
		FW_TrackThreshold_L = TRACK_TH_L;
		FW_Tapping_Ratio = TAPPING_RATIO;
		FW_Tapping_Dis_H = TAPPING_DIS_H;
		FW_Tapping_Dis_L = TAPPING_DIS_L;
		FW_PWR_State = AC_MODE;
		FW_Boundary_DY_TH = BOUNDARY_CHEAT_DY_TH;
		FW_Boundary_DisX_TH = BOUNDARY_CHEAT_DIS_X_TH;
		FW_Same_Dir_Cnt_X = SAME_DIR_CNT_X_TH;
		FW_Boundary_Range_H = BOUNDARY_CHEAT_RANGE_H;
		FW_Boundary_Range_L = BOUNDARY_CHEAT_RANGE_L;
        FW_FIR_EN = DISABLE;
        FW_Min_Fir_Cnt = MIN_FIR_CNT;
        FW_Max_Fir_Cnt = MAX_FIR_CNT;
        FW_Fir_Step_L = FIR_STEP_L;
        FW_Fir_Step_H = FIR_STEP_H;
        FW_Median_Order = MEDIAN_ORDER;
        FW_EnterDebounce[0] = EnterDB_ID0;
        FW_EnterDebounce[1] = EnterDB_ID1;
        FW_EnterDebounce[2] = EnterDB_ID2;
        FW_EnterDebounce[3] = EnterDB_ID3;
        FW_EnterDebounce[4] = EnterDB_ID4;
        FW_EnterDebounce[5] = EnterDB_ID5;
        FW_EnterDebounce[6] = EnterDB_ID6;
        FW_EnterDebounce[7] = EnterDB_ID7;
        FW_EnterDebounce[8] = EnterDB_ID8;
        FW_EnterDebounce[9] = EnterDB_ID9;
        FW_Tracking_Anti_Cross_En = ENABLE;
        FW_X_Jitter_Offset = C_X_Jitter_Offset;
        FW_X_Jitter_Maximun = C_X_Jitter_Max;
        FW_Y_Jitter_Offset = C_Y_Jitter_Offset;
        FW_Y_Jitter_Maximun = C_Y_Jitter_Max;
		FW_CrossFinNum = MAX_CROSS_FIN_NUM;

//#if DEBUG_MODE
		NVT_ALG_NOTICE("\nParam initialization:\n");
		NVT_ALG_NOTICE("FW_IIR_Distance: %d\n", (FW_IIR_Distance_H*256 + FW_IIR_Distance_L));
		NVT_ALG_NOTICE("FW_IIR_Weight: %d\n", FW_IIR_Weight);
		NVT_ALG_NOTICE("FW_IIR_Start: %d\n", FW_IIR_Start);
		NVT_ALG_NOTICE("FW_FinUP_TH: %d\n", FW_FinUP_TH);
		NVT_ALG_NOTICE("FW_TrackThreshold: %d\n", (FW_TrackThreshold_H*256 + FW_TrackThreshold_L));
		NVT_ALG_NOTICE("FW_Tapping_Ratio: %d\n", FW_Tapping_Ratio);
		NVT_ALG_NOTICE("FW_Tapping_Dis: %d\n", (FW_Tapping_Dis_H*256 + FW_Tapping_Dis_L));
		NVT_ALG_NOTICE("FW_PWR_State: %d\n", FW_PWR_State);
		NVT_ALG_NOTICE("FW_Boundary_DY_TH: %d\n", FW_Boundary_DY_TH);
		NVT_ALG_NOTICE("FW_Boundary_DisX_TH: %d\n", FW_Boundary_DisX_TH);
		NVT_ALG_NOTICE("FW_Same_Dir_Cnt_X: %d\n", FW_Same_Dir_Cnt_X);
		NVT_ALG_NOTICE("FW_Boundary_Range: %d\n", (FW_Boundary_Range_H*256 + FW_Boundary_Range_L));
        NVT_ALG_NOTICE("FW_Min_Fir_Cnt: %d\n", FW_Min_Fir_Cnt);
        NVT_ALG_NOTICE("FW_Max_Fir_Cnt: %d\n", FW_Max_Fir_Cnt);
        NVT_ALG_NOTICE("FW_Fir_Step: %d\n", (FW_Fir_Step_H*256 + FW_Fir_Step_L));
		NVT_ALG_NOTICE("FW_Median_Order: %d\n", FW_Median_Order);
        NVT_ALG_NOTICE("FW_EnterDebounce[0]: %d\n", FW_EnterDebounce[0]);
        NVT_ALG_NOTICE("FW_EnterDebounce[1]: %d\n", FW_EnterDebounce[1]);
        NVT_ALG_NOTICE("FW_EnterDebounce[2]: %d\n", FW_EnterDebounce[2]);
        NVT_ALG_NOTICE("FW_EnterDebounce[3]: %d\n", FW_EnterDebounce[3]);
        NVT_ALG_NOTICE("FW_EnterDebounce[4]: %d\n", FW_EnterDebounce[4]);
        NVT_ALG_NOTICE("FW_EnterDebounce[5]: %d\n", FW_EnterDebounce[5]);
        NVT_ALG_NOTICE("FW_EnterDebounce[6]: %d\n", FW_EnterDebounce[6]);
        NVT_ALG_NOTICE("FW_EnterDebounce[7]: %d\n", FW_EnterDebounce[7]);
        NVT_ALG_NOTICE("FW_EnterDebounce[8]: %d\n", FW_EnterDebounce[8]);
        NVT_ALG_NOTICE("FW_EnterDebounce[9]: %d\n", FW_EnterDebounce[9]);
		NVT_ALG_NOTICE("FW_Tracking_Anti_Cross_En: %d\n", FW_Tracking_Anti_Cross_En);
		NVT_ALG_NOTICE("FW_X_Jitter_Offset: %d\n", FW_X_Jitter_Offset);
		NVT_ALG_NOTICE("FW_X_Jitter_Maximun: %d\n", FW_X_Jitter_Maximun);
		NVT_ALG_NOTICE("FW_Y_Jitter_Offset: %d\n", FW_Y_Jitter_Offset);
		NVT_ALG_NOTICE("FW_Y_Jitter_Maximun: %d\n", FW_Y_Jitter_Maximun);
		NVT_ALG_NOTICE("FW_CrossFinNum: %d\n", FW_CrossFinNum);
//#endif
	}

	//parsing start
	if ( FW_PARA_UPDATE_EN && ((pData[61] & 0x01) == 0) )		//FW control Update parameter
	{
        //Update parameter every frame
        FW_PWR_State = ((pData[61]>>1) & 0x03);
		FW_IIR_Distance_L = pData[62];
        FW_IIR_Distance_H = (pData[74] & 0x0F);
		FW_IIR_Weight = (unsigned char) two_power((pData[63] & 0x0F));
		FW_IIR_Start = (pData[63]>>4);

        if (FW_FinUP_TH != (pData[64] & 0x07))
        {
            FinUP_Update = 1;
            FW_FinUP_TH = (pData[64] & 0x07);
        }
        else
            FinUP_Update = 0;

		FW_TrackThreshold_H = (pData[64]>>3);
		FW_TrackThreshold_L = pData[65];
		FW_Tapping_Ratio= (pData[66] & 0x3F);
		FW_Tapping_Dis_H = (pData[66]>>6);
		FW_Tapping_Dis_L = pData[67];
		FW_Boundary_DY_TH = pData[68];
		FW_Boundary_DisX_TH = pData[69];
		FW_Same_Dir_Cnt_X = (pData[70] & 0x07);
		FW_Boundary_Range_H = pData[70]>>3;
		FW_Boundary_Range_L = pData[71];

        if ((pData[61]&0x08) == 0)
            FW_FIR_EN = 0;
        else
            FW_FIR_EN = 1;

        //FW_FIR_EN = (pData[61]&0x08 == 0)?0:1;

        FW_Min_Fir_Cnt = (pData[72]&0x0F);
        FW_Min_Fir_Cnt = (pData[72]>>4);
        FW_Fir_Step_L = pData[73];
        FW_Fir_Step_H = pData[74]>>4;
        FW_Median_Order = ((pData[61] & 0x7F)>>4)+1;
        FW_EnterDebounce[0] = (pData[75] & 0x0F);
        FW_EnterDebounce[1] = pData[75]>>4;
        FW_EnterDebounce[2] = (pData[76] & 0x0F);
        FW_EnterDebounce[3] = pData[76]>>4;
        FW_EnterDebounce[4] = (pData[77] & 0x0F);
        FW_EnterDebounce[5] = pData[77]>>4;
        FW_EnterDebounce[6] = (pData[78] & 0x0F);
        FW_EnterDebounce[7] = pData[78]>>4;
        FW_EnterDebounce[8] = (pData[79] & 0x0F);
        FW_EnterDebounce[9] = pData[79]>>4;

        if ((pData[61]&0x80) == 0)
            FW_Tracking_Anti_Cross_En = 0;
        else
            FW_Tracking_Anti_Cross_En = 1;

        FW_X_Jitter_Offset = (pData[80] & 0x0F);
        FW_X_Jitter_Maximun = (pData[80]>>4);
        FW_Y_Jitter_Offset = (pData[81] & 0x0F);
        FW_Y_Jitter_Maximun = (pData[81]>>4);

		FW_CrossFinNum = (pData[82] & 0x0F);
		if (FW_CrossFinNum > 10)
			FW_CrossFinNum = MAX_CROSS_FIN_NUM;

//#if DEBUG_MODE
        if (FW_Update == 0)
        {
            FW_Update = 1;
    		//Debug
    		NVT_ALG_NOTICE("\nupdate parameter from FW\n");
    		NVT_ALG_NOTICE("Data:\n");
    		NVT_ALG_NOTICE("data[61]: %d\n", pData[61]);
    		NVT_ALG_NOTICE("data[62]: %d\n", pData[62]);
    		NVT_ALG_NOTICE("data[63]: %d\n", pData[63]);
    		NVT_ALG_NOTICE("data[64]: %d\n", pData[64]);
    		NVT_ALG_NOTICE("data[65]: %d\n", pData[65]);
    		NVT_ALG_NOTICE("data[66]: %d\n", pData[66]);
    		NVT_ALG_NOTICE("data[67]: %d\n", pData[67]);
            NVT_ALG_NOTICE("data[68]: %d\n", pData[68]);
            NVT_ALG_NOTICE("data[69]: %d\n", pData[69]);
            NVT_ALG_NOTICE("data[70]: %d\n", pData[70]);
            NVT_ALG_NOTICE("data[71]: %d\n", pData[71]);
            NVT_ALG_NOTICE("data[72]: %d\n", pData[72]);
            NVT_ALG_NOTICE("data[73]: %d\n", pData[73]);
            NVT_ALG_NOTICE("data[74]: %d\n", pData[74]);
            NVT_ALG_NOTICE("\nParam:\n");
    		NVT_ALG_NOTICE("FW_IIR_Distance: %d\n", (FW_IIR_Distance_H*256 + FW_IIR_Distance_L));
    		NVT_ALG_NOTICE("FW_IIR_Weight: %d\n", FW_IIR_Weight);
    		NVT_ALG_NOTICE("FW_IIR_Start: %d\n", FW_IIR_Start);
    		NVT_ALG_NOTICE("FW_FinUP_TH: %d\n", FW_FinUP_TH);
    		NVT_ALG_NOTICE("FW_TrackThreshold: %d\n", (FW_TrackThreshold_H*256 + FW_TrackThreshold_L));
    		NVT_ALG_NOTICE("FW_Tapping_Ratio: %d\n", FW_Tapping_Ratio);
    		NVT_ALG_NOTICE("FW_Tapping_Dis: %d\n", (FW_Tapping_Dis_H*256 + FW_Tapping_Dis_L));
    		NVT_ALG_NOTICE("FW_PWR_State: %d\n", FW_PWR_State);
    		NVT_ALG_NOTICE("FW_Boundary_DY_TH: %d\n", FW_Boundary_DY_TH);
    		NVT_ALG_NOTICE("FW_Boundary_DisX_TH: %d\n", FW_Boundary_DisX_TH);
    		NVT_ALG_NOTICE("FW_Same_Dir_Cnt_X: %d\n", FW_Same_Dir_Cnt_X);
    		NVT_ALG_NOTICE("FW_Boundary_Range: %d\n", (FW_Boundary_Range_H*256 + FW_Boundary_Range_L));
            NVT_ALG_NOTICE("Fir Enable: %d\n", FW_FIR_EN);
            NVT_ALG_NOTICE("FW_Min_Fir_Cnt: %d\n", FW_Min_Fir_Cnt);
            NVT_ALG_NOTICE("FW_Max_Fir_Cnt: %d\n", FW_Max_Fir_Cnt);
            NVT_ALG_NOTICE("FW_Fir_Step: %d\n", (FW_Fir_Step_H*256 + FW_Fir_Step_L));
            NVT_ALG_NOTICE("FW_Median_Order: %d\n", FW_Median_Order);
            NVT_ALG_NOTICE("FW_EnterDebounce[0]: %d\n", FW_EnterDebounce[0]);
            NVT_ALG_NOTICE("FW_EnterDebounce[1]: %d\n", FW_EnterDebounce[1]);
            NVT_ALG_NOTICE("FW_EnterDebounce[2]: %d\n", FW_EnterDebounce[2]);
            NVT_ALG_NOTICE("FW_EnterDebounce[3]: %d\n", FW_EnterDebounce[3]);
            NVT_ALG_NOTICE("FW_EnterDebounce[4]: %d\n", FW_EnterDebounce[4]);
            NVT_ALG_NOTICE("FW_EnterDebounce[5]: %d\n", FW_EnterDebounce[5]);
            NVT_ALG_NOTICE("FW_EnterDebounce[6]: %d\n", FW_EnterDebounce[6]);
            NVT_ALG_NOTICE("FW_EnterDebounce[7]: %d\n", FW_EnterDebounce[7]);
            NVT_ALG_NOTICE("FW_EnterDebounce[8]: %d\n", FW_EnterDebounce[8]);
            NVT_ALG_NOTICE("FW_EnterDebounce[9]: %d\n", FW_EnterDebounce[9]);
            NVT_ALG_NOTICE("FW_Tracking_Anti_Cross_En: %d\n", FW_Tracking_Anti_Cross_En);
			NVT_ALG_NOTICE("FW_X_Jitter_Offset: %d\n", FW_X_Jitter_Offset);
			NVT_ALG_NOTICE("FW_X_Jitter_Maximun: %d\n", FW_X_Jitter_Maximun);
			NVT_ALG_NOTICE("FW_Y_Jitter_Offset: %d\n", FW_Y_Jitter_Offset);
			NVT_ALG_NOTICE("FW_Y_Jitter_Maximun: %d\n", FW_Y_Jitter_Maximun);
			NVT_ALG_NOTICE("FW_CrossFinNum: %d\n", FW_CrossFinNum);
        }
//#endif
	}

    return;
}

//Porting by TY Chan 2013.08.08
static void Point_FIR(unsigned char index, char MIN_Fir_Cnt, char MAX_Fir_Cnt,unsigned int Fir_Step)
{
	unsigned char i, j;
	int FIR_id;
    unsigned char ID = mPointState.ID[index];
    int sumX, sumY;

    FIRPointPosX[index] = mPointState.PosX[index];
    FIRPointPosY[index] = mPointState.PosY[index];

    if (mPointState.Status[index] == STATUS_IN)
    {
        for (j=0; j<10; j++)
        {
            FIR_X[ID][j] = mPointState.PosX[index];
            FIR_Y[ID][j] = mPointState.PosY[index];
        }
		FIR_point[ID] = 0;
		FIR_id = 0;
		FIR_weight[ID] = MAX_Fir_Cnt ;
    }
    else if (mPointState.Status[index] == STATUS_MOVE)
    {
        FIR_point[ID]++;
		FIR_id = FIR_point[ID]%10;
		FIR_point[ID] = FIR_id;
		FIR_X[ID][FIR_id] = mPointState.PosX[index];
		FIR_Y[ID][FIR_id] = mPointState.PosY[index];

        if (Velocity[ID] > Fir_Step)
			FIR_weight[ID]--;
		else
			FIR_weight[ID]++;

		if (FIR_weight[ID] > MAX_Fir_Cnt)
			FIR_weight[ID] = MAX_Fir_Cnt;
		else if( FIR_weight[ID] < MIN_Fir_Cnt)
			FIR_weight[ID] = MIN_Fir_Cnt;

        sumX = mPointState.PosX[index];
        sumY = mPointState.PosY[index];
		for (i=1; i<FIR_weight[ID]; i++)
        {
			FIR_id--;
			if (FIR_id < 0)
				FIR_id = 9;

			sumX += FIR_X[ID][FIR_id];
            sumY += FIR_Y[ID][FIR_id];
		}
		FIRPointPosX[index] = sumX/FIR_weight[ID];
		FIRPointPosY[index] = sumY/FIR_weight[ID];
    }
    else if (mPointState.Status[index] == STATUS_NONE)
    {

    }
}

unsigned int getArea(unsigned char index)
{
    int i;
    unsigned int Area = 0;

    for (i=0; i<prePointState.points; i++ )
    {
        if (prePointState.ID[i] == mPointState.ID[index])
        {
            Area = prePointState.Area[i];
            break;
        }
    }

    if (i == prePointState.points)
        Area = mPointState.Area[index] + AREA_OFFSET;
    else
    {
        Area = ((Area + (unsigned int)mPointState.Area[index])/2) + AREA_OFFSET;
    }

	return (Area>255)?255:Area;
}

unsigned int getPressure(unsigned char index)
{
    int i;
    unsigned int Pressure = 0;

    for (i=0; i<prePointState.points; i++ )
    {
        if (prePointState.ID[i] == mPointState.ID[index])
        {
            Pressure = prePointState.Pressure[i];
            break;
        }
    }

    if (i == prePointState.points)
        Pressure = mPointState.Pressure[index] + PRESSURE_OFFSET;
    else
    {
        Pressure = ((Pressure + (unsigned int)mPointState.Pressure[index])/2) + PRESSURE_OFFSET;
    }

	return (Pressure>255)?255:Pressure;
}


static void MedianFilter_init(unsigned char ID, unsigned int crood_x, unsigned int crood_y)
{
    int i;

    for (i=0; i<8; i++)
    {
        Median_X[ID][i] = crood_x;
        Median_Y[ID][i] = crood_y;
    }

    return;
}


static void MedianFilter(unsigned short InX, unsigned short InY, unsigned char ID, unsigned char Order, unsigned char index)
{
    unsigned char  u8loop;
    unsigned short BufX[8], BufY[8];
    unsigned char  kk, jj;
    unsigned short tmp;

    for(u8loop =1;u8loop <8 ; u8loop++)
    {
        Median_X[ID][u8loop] = Median_X[ID][u8loop-1];
        Median_Y[ID][u8loop] = Median_Y[ID][u8loop-1];
    }

    Median_X[ID][0] = InX;
    Median_Y[ID][0] = InY;

    //--------------------------------------------
    if(Order > 8)
    {
        Order = 8;
    }
    else if(Order < 3)
    {
		MedianPointPosX[index] = InX;
    	MedianPointPosY[index] = InY;
        return;
    }

    for(kk =0; kk  <Order ; kk++)
    {
        BufX[kk] = Median_X[ID][kk];
        BufY[kk] = Median_Y[ID][kk];
    }

    for(kk = 1; kk < Order; kk++)
    {
        for(jj =0; jj < Order-1; jj++)
        {
            if(BufX[jj] > BufX[jj+1])
            {
                tmp = BufX[jj] ;
                BufX[jj] = BufX[jj+1];
                BufX[jj+1] = tmp;
            }

            if(BufY[jj] > BufY[jj+1])
            {
                tmp = BufY[jj];
                BufY[jj] = BufY[jj+1];
                BufY[jj+1] = tmp;
            }
        }
    }

	MedianPointPosX[index] = (BufX[Order>>1]);
    MedianPointPosY[index] = (BufY[Order>>1]);

    return;
}


static signed int getCrossValue(int pos1_x, int pos1_y,int pos2_x, int pos2_y, int testPos_x, int testPos_y)
{
    signed int retValue;

    retValue = (testPos_y - pos1_y)*(pos1_x - pos2_x) - (pos1_y - pos2_y)*(testPos_x - pos1_x);

    return retValue;
}


static void crossCheckHandler(void)
{
    int i, j;
    int target_ID1, target_ID2;
    int testPos_x, testPos_y, pos2_x,pos2_y;
    signed int result1, result2, result3, result4;

    if ((mPointState.points == 1) || (mPointState.points > FW_CrossFinNum))
        return;

    for (i=0; i<mPointState.points; i++)
    {
		target_ID1 = mPointState.ID[i];
		if (target_ID1 >= MAXID)
			continue;

        if ((mPointState.Status[i] != STATUS_MOVE) || (ID_Valid[target_ID1] == 0) || (Velocity[target_ID1] < 30))
            continue;

        for (j=0; j<mPointState.points; j++)
        {
			target_ID2 = mPointState.ID[j];
			if (target_ID2 >= MAXID)
				continue;

			if (target_ID2 == target_ID1)
				continue;

            if ((mPointState.Status[j] != STATUS_MOVE) || (ID_Valid[target_ID2] == 0) || (Velocity[target_ID2] < 30))
                continue;

            //Line 1
            pos2_x = getPreRawPoint(OPTION_X, target_ID1);
            pos2_y = getPreRawPoint(OPTION_Y, target_ID1);
            testPos_x = mPointState.PosX[j];
            testPos_y = mPointState.PosY[j];

            result1 = getCrossValue(mPointState.PosX[i], mPointState.PosY[i], pos2_x, pos2_y, testPos_x, testPos_y);

            testPos_x = getPreRawPoint(OPTION_X, target_ID2);
            testPos_y = getPreRawPoint(OPTION_Y, target_ID2);

            result2 = getCrossValue(mPointState.PosX[i], mPointState.PosY[i], pos2_x, pos2_y, testPos_x, testPos_y);

			if ( ((result1 > 0) && (result2 > 0)) || ((result1 < 0) && (result2 < 0)) || (result1 == 0) || (result2 == 0) )
                continue;

            //Line 2
            pos2_x = getPreRawPoint(OPTION_X, target_ID2);
            pos2_y = getPreRawPoint(OPTION_Y, target_ID2);
            testPos_x = mPointState.PosX[i];
            testPos_y = mPointState.PosY[i];

            result3 = getCrossValue(mPointState.PosX[j], mPointState.PosY[j], pos2_x, pos2_y, testPos_x, testPos_y);

            testPos_x = getPreRawPoint(OPTION_X, target_ID1);
            testPos_y = getPreRawPoint(OPTION_Y, target_ID1);

            result4 = getCrossValue(mPointState.PosX[j], mPointState.PosY[j], pos2_x, pos2_y, testPos_x, testPos_y);

			if ( ((result3 > 0) && (result4 > 0)) || ((result3 < 0) && (result4 < 0)) || (result3 == 0) || (result4 == 0) )
			  continue;

            mPointState.ID[i] = target_ID2;
            mPointState.ID[j] = target_ID1;
			//NVT_ALG_NOTICE("%d, %d, %d, %d\n",mPointState.PosX[i], mPointState.PosY[i], getPreRawPoint(OPTION_X, target_ID1), getPreRawPoint(OPTION_Y, target_ID1));
			//NVT_ALG_NOTICE("%d, %d, %d, %d\n",mPointState.PosX[j], mPointState.PosY[j], getPreRawPoint(OPTION_X, target_ID2), getPreRawPoint(OPTION_Y, target_ID2));
			//NVT_ALG_NOTICE("%d, %d, %d, %d\n", result1, result2, result3, result4);
			//NVT_ALG_NOTICE("cross !!!  %d,    %d\n", target_ID1, target_ID2);
            target_ID1 = target_ID2;
        }

    }

    return;
}

//IIR/FIR, Jitter, MedianFilter
static void smoothing(unsigned char X_Jitter_Max, unsigned char Y_Jitter_Max, unsigned char X_Jitter_Offset, unsigned char Y_Jitter_Offset, int IIR_Weight, int IIR_Start, unsigned int IIR_Distance)
{
    int i, preIndex, Dis_XY, trackedID;

    for (i=0; i<mPointState.points; i++ )
    {

        if (mPointState.Status[i] == STATUS_MOVE)
        {
            trackedID = mPointState.ID[i];
            preIndex = getPreIndex(trackedID);
            Dis_XY = Distance_Sqrt(prePointState.PosX[preIndex], prePointState.PosY[preIndex], mPointState.PosX[i], mPointState.PosY[i]);

            if (FW_FIR_EN == 1)
            {
                Point_FIR(i, FW_Min_Fir_Cnt, FW_Max_Fir_Cnt, (FW_Fir_Step_H*256 + FW_Fir_Step_L));

				IIRPointState.PosX[i] = FIRPointPosX[i];
            	IIRPointState.PosY[i] = FIRPointPosY[i];
            }
            else
            {
                if(Dis_XY > IIR_Distance)
                {
        	         IIRPointState.PosX[i] = mPointState.PosX[i];
            	     IIRPointState.PosY[i] = mPointState.PosY[i];
        	         ReportPointState.PosX[i] = mPointState.PosX[i];
            	     ReportPointState.PosY[i] = mPointState.PosY[i];
                }
                else
                {
        	         //Dynamic IIR
        	         IIRPointState.PosX[i] = (mPointState.PosX[i]*(IIR_Distance*IIR_Start+Dis_XY*(IIR_Weight-IIR_Start))+preIIRPointState.PosX[preIndex]*(IIR_Distance*(IIR_Weight-IIR_Start)-Dis_XY*(IIR_Weight-IIR_Start)))/(IIR_Distance*IIR_Weight);
        	         IIRPointState.PosY[i] = (mPointState.PosY[i]*(IIR_Distance*IIR_Start+Dis_XY*(IIR_Weight-IIR_Start))+preIIRPointState.PosY[preIndex]*(IIR_Distance*(IIR_Weight-IIR_Start)-Dis_XY*(IIR_Weight-IIR_Start)))/(IIR_Distance*IIR_Weight);
                }
            }

        #if G_FINGER_EN
            lastVector_X[trackedID] = IIRPointState.PosX[i] - preReportPointState.PosX[preIndex];
            lastVector_Y[trackedID] = IIRPointState.PosY[i] - preReportPointState.PosY[preIndex];
        #endif

			//Jitter
            if((IIRPointState.PosX[i]>X_Jitter_Max+preReportPointState.PosX[i])||(preReportPointState.PosX[preIndex]>X_Jitter_Max+IIRPointState.PosX[i]))
            {
                 ReportPointState.PosX[i]=IIRPointState.PosX[i];
                 ReportPointState.PosY[i]=IIRPointState.PosY[i];
            }
            else if((IIRPointState.PosY[i]>Y_Jitter_Max+preReportPointState.PosY[i])||(preReportPointState.PosY[preIndex]>Y_Jitter_Max+IIRPointState.PosY[i]))
            {
                 ReportPointState.PosX[i]=IIRPointState.PosX[i];
                 ReportPointState.PosY[i]=IIRPointState.PosY[i];
            }
            else
            {
                 if(((IIRPointState.PosX[i]>X_Jitter_Offset+preReportPointState.PosX[preIndex])||(preReportPointState.PosX[preIndex]>X_Jitter_Offset+IIRPointState.PosX[i]))&&((IIRPointState.PosY[i]>Y_Jitter_Offset+preReportPointState.PosY[preIndex])||(preReportPointState.PosY[preIndex]>Y_Jitter_Offset+IIRPointState.PosY[i])))
                 {
                     ReportPointState.PosX[i]=IIRPointState.PosX[i];
                     ReportPointState.PosY[i]=IIRPointState.PosY[i];
                 }
                 else
                 {
                     ReportPointState.PosX[i]=preReportPointState.PosX[preIndex];
                     ReportPointState.PosY[i]=preReportPointState.PosY[preIndex];
                 }
            }

			MedianFilter((unsigned short)ReportPointState.PosX[i], (unsigned short)ReportPointState.PosY[i], trackedID, FW_Median_Order, i);
			ReportPointState.PosX[i] = MedianPointPosX[i];
			ReportPointState.PosY[i] = MedianPointPosY[i];

        }
    }

	return;
}
