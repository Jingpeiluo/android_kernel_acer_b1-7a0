/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MTK_BATTERY_METER_TABLE_H
#define _MTK_BATTERY_METER_TABLE_H

/* ============================================================*/
/* define*/
/* ============================================================*/
#define BAT_NTC_10 1
#define BAT_NTC_47 0
#define BAT_NTC_100 0

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R             16900
#define RBAT_PULL_DOWN_R           30000
#endif
#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R             61900
#define RBAT_PULL_DOWN_R           100000
#endif
#if (BAT_NTC_100 == 1)
#define RBAT_PULL_UP_R             24000
#define RBAT_PULL_DOWN_R           100000000
#endif
#define RBAT_PULL_UP_VOLT          1800


/* ============================================================*/
/* ENUM*/
/* ============================================================*/

/* ============================================================*/
/* structure*/
/* ============================================================*/

/* ============================================================*/
/* typedef*/
/* ============================================================*/
typedef struct _BATTERY_PROFILE_STRUCT {
	signed int percentage;
	signed int voltage;
} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT {
	signed int resistance;
	signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum {
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

/* ============================================================*/
/* External Variables*/
/* ============================================================*/

/* ============================================================*/
/* External function*/
/* ============================================================*/

/* ============================================================*/
/* <DOD, Battery_Voltage> Table*/
/* ============================================================*/
#if (BAT_NTC_10 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
#if 0//ori
		{-20, 76241},
		{-15, 58649},
		{-10, 45569},
		{ -5, 35735},
		{  0, 28271},
		{  5, 22551},
		{ 10, 18136},
		{ 15, 14696},
		{ 20, 11997},
		{ 25, 10000},
		{ 30, 8365},
		{ 35, 7011},
		{ 40, 5951},
		{ 45, 4992},
		{ 50, 4217},
		{ 55, 3579},
		{ 60, 3051}
#else
		{-20, 74354},
        {-15, 57626},
        {-10, 45068},
        {-5,  35548},
        {0 ,	28267},
        {5 ,	22650},
        {10 ,	18280},
        {15 ,	14855},
        {20 ,	12150},
        {25 ,	10000},
        {30 ,	8279 },
        {35 ,	6892 },
        {40 ,	5768 },
        {45 ,	4852},
        {50 ,	4101},
        {55 ,	3482 },
        {60 ,	2970},	
#endif	
};
#endif

#if (BAT_NTC_47 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20, 483954},
		{-15, 360850},
		{-10, 271697},
		{ -5, 206463},
		{  0, 158214},
		{  5, 122259},
		{ 10, 95227},
		{ 15, 74730},
		{ 20, 59065},
		{ 25, 47000},
		{ 30, 37643},
		{ 35, 30334},
		{ 40, 24591},
		{ 45, 20048},
		{ 50, 16433},
		{ 55, 13539},
		{ 60, 11210}
};
#endif

#if (BAT_NTC_100 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
		{-20, 1151037},
		{-15, 846579},
		{-10, 628988},
		{ -5, 471632},
		{  0, 357012},
		{  5, 272500},
		{ 10, 209710},
		{ 15, 162651},
		{ 20, 127080},
		{ 25, 100000},
		{ 30, 79222},
		{ 35, 63167},
		{ 40, 50677},
		{ 45, 40904},
		{ 50, 33195},
		{ 55, 27091},
		{ 60, 22224}
};
#endif
/* T0 -10C*/
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
#if 0
	{0		  ,  4357},
	{1		  ,  4327},
	{2		  ,  4305},
	{3		  ,  4289},
	{4		  ,  4273},
	{5		  ,  4260},
	{6		  ,  4247},
	{8		  ,  4233},
	{9		  ,  4221},
	{10 	  ,  4209},
	{11 	  ,  4196},
	{12 	  ,  4184},
	{13 	  ,  4172},
	{14 	  ,  4161},
	{15 	  ,  4149},
	{16 	  ,  4136},
	{17 	  ,  4125},
	{18 	  ,  4113},
	{19 	  ,  4104},
	{20 	  ,  4093},
	{21 	  ,  4083},
	{23 	  ,  4075},
	{24 	  ,  4066},
	{25 	  ,  4055},
	{26 	  ,  4040},
	{27 	  ,  4022},
	{28 	  ,  4001},
	{29 	  ,  3984},
	{30 	  ,  3970},
	{31 	  ,  3957},
	{32 	  ,  3948},
	{33 	  ,  3939},
	{34 	  ,  3933},
	{35 	  ,  3925},
	{36 	  ,  3919},
	{38 	  ,  3913},
	{39 	  ,  3907},
	{40 	  ,  3901},
	{41 	  ,  3893},
	{42 	  ,  3886},
	{43 	  ,  3879},
	{44 	  ,  3872},
	{45 	  ,  3865},
	{46 	  ,  3858},
	{47 	  ,  3852},
	{48 	  ,  3846},
	{49 	  ,  3840},
	{50 	  ,  3833},
	{52 	  ,  3828},
	{53 	  ,  3823},
	{54 	  ,  3817},
	{55 	  ,  3814},
	{56 	  ,  3809},
	{57 	  ,  3804},
	{58 	  ,  3801},
	{59 	  ,  3796},
	{60 	  ,  3793},
	{61 	  ,  3789},
	{62 	  ,  3786},
	{63 	  ,  3783},
	{64 	  ,  3780},
	{65 	  ,  3778},
	{67 	  ,  3777},
	{68 	  ,  3774},
	{69 	  ,  3771},
	{70 	  ,  3770},
	{71 	  ,  3769},
	{72 	  ,  3766},
	{73 	  ,  3764},
	{74 	  ,  3761},
	{75 	  ,  3759},
	{76 	  ,  3756},
	{77 	  ,  3754},
	{78 	  ,  3751},
	{79 	  ,  3748},
	{80 	  ,  3745},
	{82 	  ,  3742},
	{83 	  ,  3737},
	{84 	  ,  3734},
	{85 	  ,  3729},
	{86 	  ,  3724},
	{87 	  ,  3719},
	{88 	  ,  3715},
	{89 	  ,  3712},
	{90 	  ,  3706},
	{91 	  ,  3701},
	{92 	  ,  3696},
	{93 	  ,  3691},
	{94 	  ,  3684},
	{96 	  ,  3675},
	{97 	  ,  3664},
	{98 	  ,  3644},
	{99 	  ,  3616},
	{100	  ,  3500}
#else
{0   ,4190 },
{1   ,4173 },
{2   ,4159 },
{3   ,4150 },
{4   ,4140 },
{5   ,4130 },
{6   ,4120 },
{7   ,4113 },
{8   ,4096 },
{9   ,4086 },
{10  ,4078 },
{11  ,4070 },
{12  ,4061 },
{13  ,4052 },
{14  ,4043 },
{15  ,4034 },
{16  ,4027 },
{17  ,4019 },
{18  ,4010 },
{19  ,4002 },
{20  ,3993 },
{21  ,3986 },
{22  ,3979 },
{23  ,3961 },
{24  ,3955 },
{25  ,3947 },
{26  ,3939 },
{27  ,3932 },
{28  ,3924 },
{29  ,3919 },
{30  ,3912 },
{31  ,3905 },
{32  ,3899 },
{33  ,3893 },
{35  ,3882 },
{36  ,3876 },
{37  ,3867 },
{38  ,3861 },
{39  ,3856 },
{40  ,3852 },
{41  ,3847 },
{43  ,3840 },
{44  ,3834 },
{45  ,3831 },
{46  ,3827 },
{47  ,3823 },
{48  ,3819 },
{49  ,3815 },
{50  ,3814 },
{51  ,3806 },
{52  ,3803 },
{53  ,3799 },
{54  ,3797 },
{55  ,3794 },
{56  ,3792 },
{58  ,3787 },
{59  ,3784 },
{60  ,3781 },
{61  ,3779 },
{64  ,3772 },
{65  ,3771 },
{66  ,3766 },
{67  ,3765 },
{68  ,3762 },
{69  ,3761 },
{70  ,3759 },
{71  ,3756 },
{72  ,3755 },
{74  ,3749 },
{75  ,3747 },
{76  ,3744 },
{77  ,3740 },
{78  ,3738 },
{79  ,3735 },
{80  ,3726 },
{81  ,3721 },
{82  ,3716 },
{84  ,3708 },
{85  ,3702 },
{86  ,3698 },
{87  ,3695 },
{88  ,3691 },
{89  ,3687 },
{90  ,3682 },
{91  ,3679 },
{92  ,3675 },
{93  ,3669 },
{94  ,3660 },
{95  ,3628 },
{96  ,3600 },
{97  ,3566 },
{98  ,3527 },
{99  ,3505 },
{100 ,3500 }
#endif	
};

/* T1 0C*/
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
#if 0	
	{0		  ,  4337},
	{1		  ,  4313},
	{2		  ,  4297},
	{3		  ,  4282},
	{4		  ,  4268},
	{5		  ,  4256},
	{6		  ,  4242},
	{8		  ,  4230},
	{9		  ,  4220},
	{10 	  ,  4206},
	{11 	  ,  4194},
	{12 	  ,  4183},
	{13 	  ,  4172},
	{14 	  ,  4161},
	{15 	  ,  4149},
	{16 	  ,  4138},
	{17 	  ,  4125},
	{18 	  ,  4115},
	{19 	  ,  4104},
	{20 	  ,  4095},
	{22 	  ,  4086},
	{23 	  ,  4080},
	{24 	  ,  4072},
	{25 	  ,  4061},
	{26 	  ,  4045},
	{27 	  ,  4025},
	{28 	  ,  4006},
	{29 	  ,  3991},
	{30 	  ,  3980},
	{31 	  ,  3970},
	{32 	  ,  3960},
	{33 	  ,  3952},
	{34 	  ,  3945},
	{35 	  ,  3939},
	{37 	  ,  3933},
	{38 	  ,  3925},
	{39 	  ,  3917},
	{40 	  ,  3908},
	{41 	  ,  3901},
	{42 	  ,  3891},
	{43 	  ,  3883},
	{44 	  ,  3875},
	{45 	  ,  3868},
	{46 	  ,  3859},
	{47 	  ,  3853},
	{48 	  ,  3848},
	{49 	  ,  3842},
	{51 	  ,  3837},
	{52 	  ,  3831},
	{53 	  ,  3826},
	{54 	  ,  3820},
	{55 	  ,  3816},
	{56 	  ,  3812},
	{57 	  ,  3807},
	{58 	  ,  3804},
	{59 	  ,  3801},
	{60 	  ,  3797},
	{61 	  ,  3794},
	{62 	  ,  3791},
	{63 	  ,  3787},
	{65 	  ,  3786},
	{66 	  ,  3783},
	{67 	  ,  3780},
	{68 	  ,  3779},
	{69 	  ,  3778},
	{70 	  ,  3776},
	{71 	  ,  3776},
	{72 	  ,  3774},
	{73 	  ,  3771},
	{74 	  ,  3770},
	{75 	  ,  3767},
	{76 	  ,  3764},
	{77 	  ,  3761},
	{79 	  ,  3759},
	{80 	  ,  3755},
	{81 	  ,  3753},
	{82 	  ,  3747},
	{83 	  ,  3743},
	{84 	  ,  3737},
	{85 	  ,  3731},
	{86 	  ,  3726},
	{87 	  ,  3717},
	{88 	  ,  3711},
	{89 	  ,  3704},
	{90 	  ,  3700},
	{91 	  ,  3696},
	{93 	  ,  3694},
	{94 	  ,  3689},
	{95 	  ,  3685},
	{96 	  ,  3672},
	{97 	  ,  3643},
	{98 	  ,  3590},
	{99 	  ,  3513},
	{100	  ,  3500}
#else
{0   ,4200 },
{1   ,4185 },
{2   ,4171 },
{3   ,4161 },
{4   ,4151 },
{5   ,4141 },
{6   ,4131 },
{7   ,4115 },
{8   ,4106 },
{9   ,4099 },
{10  ,4090 },
{11  ,4081 },
{12  ,4072 },
{13  ,4064 },
{14  ,4056 },
{15  ,4048 },
{16  ,4040 },
{17  ,4031 },
{18  ,4025 },
{19  ,4017 },
{20  ,4001 },
{21  ,3996 },
{22  ,3988 },
{23  ,3981 },
{24  ,3974 },
{25  ,3966 },
{26  ,3959 },
{27  ,3953 },
{28  ,3946 },
{29  ,3938 },
{30  ,3933 },
{31  ,3926 },
{32  ,3911 },
{33  ,3905 },
{34  ,3899 },
{35  ,3891 },
{36  ,3886 },
{37  ,3880 },
{38  ,3874 },
{39  ,3868 },
{40  ,3863 },
{41  ,3857 },
{42  ,3852 },
{43  ,3848 },
{44  ,3838 },
{45  ,3833 },
{46  ,3829 },
{47  ,3826 },
{48  ,3822 },
{49  ,3817 },
{50  ,3814 },
{51  ,3811 },
{52  ,3807 },
{53  ,3805 },
{54  ,3802 },
{55  ,3798 },
{56  ,3795 },
{57  ,3791 },
{59  ,3786 },
{60  ,3785 },
{61  ,3782 },
{62  ,3779 },
{65  ,3775 },
{66  ,3772 },
{67  ,3770 },
{69  ,3765 },
{71  ,3763 },
{72  ,3760 },
{73  ,3758 },
{75  ,3753 },
{77  ,3748 },
{78  ,3744 },
{79  ,3742 },
{80  ,3738 },
{81  ,3733 },
{82  ,3724 },
{83  ,3717 },
{84  ,3712 },
{85  ,3706 },
{86  ,3699 },
{87  ,3694 },
{88  ,3688 },
{89  ,3684 },
{90  ,3680 },
{91  ,3678 },
{92  ,3674 },
{93  ,3669 },
{94  ,3658 },
{95  ,3642 },
{96  ,3615 },
{97  ,3578 },
{98  ,3532 },
{99  ,3505 },
{100 ,3500 }
#endif	
};

/* T2 25C*/
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
#if 0	
	{0		,  4334},
	{1		,  4309},
	{2		,  4299},
	{3		,  4286},
	{4		,  4267},
	{5		,  4258},
	{6		,  4248},
	{8		,  4224},
	{9		,  4213},
	{10 	,  4200},
	{11 	,  4189},
	{12 	,  4179},
	{13 	,  4165},
	{14 	,  4156},
	{15 	,  4142},
	{16 	,  4132},
	{17 	,  4123},
	{18 	,  4109},
	{19 	,  4103},
	{20 	,  4092},
	{21 	,  4081},
	{23 	,  4066},
	{24 	,  4058},
	{25 	,  4049},
	{26 	,  4037},
	{27 	,  4018},
	{28 	,  4007},
	{29 	,  3998},
	{30 	,  3988},
	{31 	,  3981},
	{32 	,  3977},
	{33 	,  3970},
	{34 	,  3964},
	{35 	,  3958},
	{37 	,  3942},
	{38 	,  3936},
	{39 	,  3926},
	{40 	,  3924},
	{41 	,  3912},
	{42 	,  3900},
	{43 	,  3891},
	{44 	,  3881},
	{45 	,  3869},
	{46 	,  3860},
	{47 	,  3855},
	{48 	,  3846},
	{49 	,  3841},
	{51 	,  3830},
	{52 	,  3826},
	{53 	,  3822},
	{54 	,  3816},
	{55 	,  3814},
	{56 	,  3811},
	{57 	,  3806},
	{58 	,  3803},
	{59 	,  3802},
	{60 	,  3797},
	{61 	,  3794},
	{62 	,  3791},
	{63 	,  3787},
	{64 	,  3785},
	{66 	,  3780},
	{67 	,  3778},
	{68 	,  3776},
	{69 	,  3774},
	{70 	,  3773},
	{71 	,  3769},
	{72 	,  3766},
	{73 	,  3763},
	{74 	,  3760},
	{75 	,  3757},
	{76 	,  3754},
	{77 	,  3747},
	{78 	,  3744},
	{80 	,  3738},
	{81 	,  3734},
	{82 	,  3728},
	{83 	,  3719},
	{84 	,  3714},
	{85 	,  3709},
	{86 	,  3703},
	{87 	,  3695},
	{88 	,  3688},
	{89 	,  3686},
	{90 	,  3686},
	{91 	,  3685},
	{92 	,  3682},
	{93 	,  3681},
	{95 	,  3652},
	{96 	,  3613},
	{97 	,  3565},
	{98 	,  3550},
	{99 	,  3540},
	{100	,  3500}
#else
{0   ,4197 },
{1   ,4184 },
{2   ,4173 },
{3   ,4162 },
{4   ,4151 },
{5   ,4141 },
{6   ,4134 },
{7   ,4114 },
{8   ,4105 },
{9   ,4097 },
{10  ,4089 },
{11  ,4080 },
{12  ,4072 },
{13  ,4062 },
{14  ,4056 },
{15  ,4047 },
{16  ,4039 },
{17  ,4033 },
{18  ,4026 },
{19  ,4011 },
{20  ,4003 },
{21  ,3997 },
{22  ,3991 },
{23  ,3983 },
{24  ,3978 },
{25  ,3972 },
{26  ,3963 },
{27  ,3959 },
{28  ,3953 },
{29  ,3948 },
{30  ,3942 },
{31  ,3930 },
{32  ,3925 },
{33  ,3919 },
{34  ,3913 },
{35  ,3908 },
{36  ,3902 },
{37  ,3897 },
{38  ,3890 },
{39  ,3884 },
{40  ,3879 },
{41  ,3873 },
{42  ,3864 },
{43  ,3859 },
{45  ,3842 },
{46  ,3837 },
{47  ,3833 },
{48  ,3828 },
{49  ,3823 },
{51  ,3815 },
{52  ,3813 },
{53  ,3809 },
{54  ,3806 },
{55  ,3802 },
{56  ,3797 },
{57  ,3795 },
{58  ,3791 },
{60  ,3786 },
{61  ,3783 },
{62  ,3781 },
{63  ,3780 },
{64  ,3778 },
{65  ,3776 },
{67  ,3770 },
{68  ,3769 },
{69  ,3766 },
{71  ,3764 },
{73  ,3760 },
{74  ,3758 },
{76  ,3752 },
{77  ,3746 },
{78  ,3742 },
{79  ,3737 },
{80  ,3729 },
{81  ,3726 },
{82  ,3721 },
{83  ,3716 },
{84  ,3711 },
{85  ,3706 },
{86  ,3700 },
{87  ,3693 },
{88  ,3685 },
{89  ,3678 },
{90  ,3674 },
{91  ,3671 },
{92  ,3652 },
{93  ,3649 },
{94  ,3645 },
{95  ,3646 },
{96  ,3625 },
{97  ,3589 },
{98  ,3544 },
{99  ,3505 },
{100 ,3500 }
#endif	
};

/* T3 50C*/
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
#if 0	
	{0	  ,  4325},
	{1	  ,  4312},
	{2	  ,  4297},
	{3	  ,  4282},
	{4	  ,  4269},
	{5	  ,  4258},
	{6	  ,  4245},
	{8	  ,  4233},
	{9	  ,  4222},
	{10   ,  4209},
	{11   ,  4198},
	{12   ,  4186},
	{13   ,  4176},
	{14   ,  4166},
	{15   ,  4153},
	{16   ,  4143},
	{17   ,  4132},
	{18   ,  4121},
	{19   ,  4109},
	{20   ,  4099},
	{21   ,  4087},
	{23   ,  4077},
	{24   ,  4066},
	{25   ,  4057},
	{26   ,  4047},
	{27   ,  4037},
	{28   ,  4027},
	{29   ,  4018},
	{30   ,  4008},
	{31   ,  3999},
	{32   ,  3990},
	{33   ,  3983},
	{34   ,  3974},
	{35   ,  3967},
	{37   ,  3957},
	{38   ,  3950},
	{39   ,  3942},
	{40   ,  3935},
	{41   ,  3926},
	{42   ,  3917},
	{43   ,  3908},
	{44   ,  3896},
	{45   ,  3882},
	{46   ,  3871},
	{47   ,  3863},
	{48   ,  3855},
	{49   ,  3850},
	{51   ,  3843},
	{52   ,  3839},
	{53   ,  3833},
	{54   ,  3830},
	{55   ,  3824},
	{56   ,  3819},
	{57   ,  3815},
	{58   ,  3812},
	{59   ,  3807},
	{60   ,  3805},
	{61   ,  3800},
	{62   ,  3797},
	{63   ,  3795},
	{64   ,  3791},
	{66   ,  3789},
	{67   ,  3785},
	{68   ,  3783},
	{69   ,  3780},
	{70   ,  3778},
	{71   ,  3774},
	{72   ,  3768},
	{73   ,  3761},
	{74   ,  3755},
	{75   ,  3751},
	{76   ,  3746},
	{77   ,  3743},
	{78   ,  3740},
	{80   ,  3735},
	{81   ,  3732},
	{82   ,  3729},
	{83   ,  3724},
	{84   ,  3718},
	{85   ,  3709},
	{86   ,  3704},
	{87   ,  3698},
	{88   ,  3689},
	{89   ,  3680},
	{90   ,  3680},
	{91   ,  3679},
	{92   ,  3679},
	{93   ,  3676},
	{95   ,  3673},
	{96   ,  3663},
	{97   ,  3627},
	{98   ,  3577},
	{99   ,  3510},
	{100  ,  3500}
#else
{0   ,4194 },
{1   ,4181 },
{2   ,4170 },
{3   ,4160 },
{4   ,4149 },
{5   ,4138 },
{6   ,4128 },
{7   ,4121 },
{8   ,4111 },
{9   ,4093 },
{10  ,4086 },
{11  ,4077 },
{12  ,4069 },
{13  ,4060 },
{14  ,4052 },
{15  ,4045 },
{16  ,4036 },
{17  ,4030 },
{18  ,4022 },
{19  ,4015 },
{20  ,4007 },
{21  ,4000 },
{22  ,3990 },
{23  ,3986 },
{24  ,3973 },
{25  ,3966 },
{27  ,3954 },
{28  ,3948 },
{29  ,3942 },
{30  ,3938 },
{31  ,3929 },
{33  ,3919 },
{34  ,3914 },
{35  ,3909 },
{36  ,3903 },
{37  ,3900 },
{38  ,3895 },
{39  ,3888 },
{40  ,3878 },
{41  ,3871 },
{42  ,3862 },
{43  ,3854 },
{44  ,3849 },
{45  ,3841 },
{46  ,3835 },
{47  ,3831 },
{48  ,3828 },
{49  ,3822 },
{50  ,3817 },
{51  ,3814 },
{53  ,3807 },
{54  ,3806 },
{55  ,3799 },
{56  ,3794 },
{58  ,3790 },
{59  ,3787 },
{60  ,3785 },
{61  ,3782 },
{63  ,3777 },
{64  ,3774 },
{65  ,3776 },
{66  ,3772 },
{67  ,3769 },
{69  ,3767 },
{70  ,3764 },
{71  ,3755 },
{72  ,3749 },
{73  ,3745 },
{74  ,3739 },
{75  ,3737 },
{76  ,3733 },
{77  ,3729 },
{78  ,3725 },
{79  ,3720 },
{80  ,3716 },
{81  ,3713 },
{82  ,3709 },
{84  ,3699 },
{85  ,3694 },
{86  ,3682 },
{87  ,3673 },
{88  ,3664 },
{89  ,3660 },
{90  ,3658 },
{91  ,3657 },
{92  ,3653 },
{93  ,3650 },
{94  ,3648 },
{95  ,3640 },
{96  ,3617 },
{97  ,3586 },
{98  ,3545 },
{99  ,3505 },
{100 ,3500 }
#endif	
};

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3*/
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0},
	{0		,  0}
};

/* ============================================================*/
/* <Rbat, Battery_Voltage> Table*/
/* ============================================================*/
/* T0 -10C*/
R_PROFILE_STRUCT r_profile_t0[] = {
#if 0	
	{673       ,  4357},
	{673	   ,  4327},
	{672	   ,  4305},
	{669	   ,  4289},
	{666	   ,  4273},
	{660	   ,  4260},
	{656	   ,  4247},
	{651	   ,  4233},
	{647	   ,  4221},
	{642	   ,  4209},
	{637	   ,  4196},
	{631	   ,  4184},
	{627	   ,  4172},
	{621	   ,  4161},
	{563	   ,  4149},
	{614	   ,  4136},
	{609	   ,  4125},
	{605	   ,  4113},
	{603	   ,  4104},
	{600	   ,  4093},
	{595	   ,  4083},
	{595	   ,  4075},
	{594	   ,  4066},
	{591	   ,  4055},
	{587	   ,  4040},
	{581	   ,  4022},
	{576	   ,  4001},
	{571	   ,  3984},
	{571	   ,  3970},
	{569	   ,  3957},
	{567	   ,  3948},
	{533	   ,  3939},
	{570	   ,  3933},
	{569	   ,  3925},
	{569	   ,  3919},
	{569	   ,  3913},
	{569	   ,  3907},
	{571       ,  3901},
	{569	   ,  3893},
	{568	   ,  3886},
	{569	   ,  3879},
	{568	   ,  3872},
	{568	   ,  3865},
	{569	   ,  3858},
	{570	   ,  3852},
	{570	   ,  3846},
	{570	   ,  3840},
	{572	   ,  3833},
	{573	   ,  3828},
	{573	   ,  3823},
	{574	   ,  3817},
	{550	   ,  3814},
	{577	   ,  3809},
	{581	   ,  3804},
	{582	   ,  3801},
	{582	   ,  3796},
	{585	   ,  3793},
	{586	   ,  3789},
	{590	   ,  3786},
	{591	   ,  3783},
	{593	   ,  3780},
	{595	   ,  3778},
	{598	   ,  3777},
	{602	   ,  3774},
	{604	   ,  3771},
	{607	   ,  3770},
	{614	   ,  3769},
	{617	   ,  3766},
	{621	   ,  3764},
	{627	   ,  3761},
	{632	   ,  3759},
	{636	   ,  3756},
	{642	   ,  3754},
	{650	   ,  3751},
	{657	   ,  3748},
	{666	   ,  3745},
	{676	   ,  3742},
	{684	   ,  3737},
	{695	   ,  3734},
	{707	   ,  3729},
	{719	   ,  3724},
	{732	   ,  3719},
	{748	   ,  3715},
	{766	   ,  3712},
	{782	   ,  3706},
	{802	   ,  3701},
	{824	   ,  3696},
	{845	   ,  3691},
	{870	   ,  3684},
	{837	   ,  3675},
	{929	   ,  3664},
	{957	   ,  3644},
	{993	   ,  3616},
	{1045	   ,  3500}
#else
{101  ,4190 },
{341  ,4173 },
{343  ,4159 },
{348  ,4150 },
{350  ,4140 },
{352  ,4130 },
{353  ,4120 },
{359  ,4113 },
{365  ,4096 },
{364  ,4086 },
{364  ,4078 },
{366  ,4070 },
{366  ,4061 },
{365  ,4052 },
{366  ,4043 },
{364  ,4034 },
{368  ,4027 },
{368  ,4019 },
{364  ,4010 },
{365  ,4002 },
{362  ,3993 },
{364  ,3986 },
{365  ,3979 },
{359  ,3961 },
{359  ,3955 },
{358  ,3947 },
{358  ,3939 },
{356  ,3932 },
{352  ,3924 },
{355  ,3919 },
{355  ,3912 },
{355  ,3905 },
{354  ,3899 },
{353  ,3893 },
{355  ,3882 },
{353  ,3876 },
{358  ,3867 },
{356  ,3861 },
{357  ,3856 },
{357  ,3852 },
{359  ,3847 },
{363  ,3840 },
{361  ,3834 },
{364  ,3831 },
{363  ,3827 },
{366  ,3823 },
{366  ,3819 },
{366  ,3815 },
{370  ,3814 },
{372  ,3806 },
{374  ,3803 },
{373  ,3799 },
{378  ,3797 },
{381  ,3794 },
{380  ,3792 },
{385  ,3787 },
{386  ,3784 },
{386  ,3781 },
{388  ,3779 },
{392  ,3772 },
{397  ,3771 },
{400  ,3766 },
{404  ,3765 },
{403  ,3762 },
{406  ,3761 },
{409  ,3759 },
{409  ,3756 },
{412  ,3755 },
{417  ,3749 },
{420  ,3747 },
{423  ,3744 },
{421  ,3740 },
{431  ,3738 },
{432  ,3735 },
{440  ,3726 },
{440  ,3721 },
{447  ,3716 },
{460  ,3708 },
{466  ,3702 },
{473  ,3698 },
{489  ,3695 },
{502  ,3691 },
{519  ,3687 },
{542  ,3682 },
{571  ,3679 },
{609  ,3675 },
{657  ,3669 },
{718  ,3660 },
{916  ,3628 },
{1083 ,3600 },
{1298 ,3566 },
{1556 ,3527 },
{1690 ,3485 },
{1732 ,3395 }
#endif	
};
/* T1 0C*/
R_PROFILE_STRUCT r_profile_t1[] = {
#if 0	
	{323	  ,  4337},
	{323	  ,  4313},
	{330	  ,  4297},
	{323	  ,  4282},
	{329	  ,  4268},
	{328	  ,  4256},
	{325	  ,  4242},
	{325	  ,  4230},
	{325	  ,  4220},
	{322	  ,  4206},
	{320	  ,  4194},
	{319	  ,  4183},
	{318	  ,  4172},
	{317	  ,  4161},
	{316	  ,  4149},
	{315	  ,  4138},
	{313	  ,  4125},
	{267	  ,  4115},
	{311	  ,  4104},
	{312	  ,  4095},
	{310	  ,  4086},
	{312	  ,  4080},
	{313	  ,  4072},
	{314	  ,  4061},
	{311	  ,  4045},
	{308	  ,  4025},
	{304	  ,  4006},
	{302	  ,  3991},
	{301	  ,  3980},
	{301	  ,  3970},
	{299	  ,  3960},
	{299	  ,  3952},
	{297	  ,  3945},
	{297	  ,  3939},
	{296	  ,  3933},
	{296	  ,  3925},
	{295	  ,  3917},
	{292	  ,  3908},
	{291	  ,  3901},
	{290	  ,  3891},
	{289	  ,  3883},
	{289	  ,  3875},
	{289	  ,  3868},
	{288	  ,  3859},
	{288	  ,  3853},
	{290	  ,  3848},
	{290	  ,  3842},
	{291	  ,  3837},
	{292	  ,  3831},
	{292	  ,  3826},
	{292	  ,  3820},
	{294	  ,  3816},
	{295	  ,  3812},
	{295	  ,  3807},
	{297	  ,  3804},
	{297	  ,  3801},
	{299	  ,  3797},
	{301	  ,  3794},
	{302	  ,  3791},
	{302	  ,  3787},
	{305	  ,  3786},
	{307	  ,  3783},
	{307	  ,  3780},
	{310	  ,  3779},
	{312	  ,  3778},
	{313	  ,  3776},
	{316	  ,  3776},
	{318	  ,  3774},
	{319	  ,  3771},
	{323	  ,  3770},
	{324	  ,  3767},
	{327	  ,  3764},
	{329	  ,  3761},
	{333	  ,  3759},
	{335	  ,  3755},
	{340	  ,  3753},
	{339	  ,  3747},
	{348	  ,  3743},
	{351	  ,  3737},
	{357	  ,  3731},
	{364	  ,  3726},
	{367	  ,  3717},
	{375	  ,  3711},
	{382	  ,  3704},
	{395	  ,  3700},
	{409	  ,  3696},
	{428	  ,  3694},
	{450	  ,  3689},
	{480	  ,  3685},
	{515	  ,  3672},
	{552	  ,  3643},
	{599	  ,  3590},
	{673	  ,  3513},
	{793	  ,  3500}
#else
{206  , 4200  },
{206  , 4185  },
{205  , 4171  },
{210  , 4161  },
{208  , 4151  },
{209  , 4141  },
{211  , 4131  },
{219  , 4115  },
{220  , 4106  },
{224  , 4099  },
{227  , 4090  },
{226  , 4081  },
{227  , 4072  },
{229  , 4064  },
{232  , 4056  },
{231  , 4048  },
{231  , 4040  },
{233  , 4031  },
{235  , 4025  },
{237  , 4017  },
{235  , 4001  },
{238  , 3996  },
{239  , 3988  },
{237  , 3981  },
{238  , 3974  },
{236  , 3966  },
{237  , 3959  },
{237  , 3953  },
{236  , 3946  },
{235  , 3938  },
{236  , 3933  },
{234  , 3926  },
{231  , 3911  },
{231  , 3905  },
{229  , 3899  },
{227  , 3891  },
{225  , 3886  },
{224  , 3880  },
{225  , 3874  },
{224  , 3868  },
{222  , 3863  },
{218  , 3857  },
{220  , 3852  },
{221  , 3848  },
{219  , 3838  },
{217  , 3833  },
{219  , 3829  },
{220  , 3826  },
{221  , 3822  },
{218  , 3817  },
{218  , 3814  },
{220  , 3811  },
{221  , 3807  },
{222  , 3805  },
{221  , 3802  },
{223  , 3798  },
{221  , 3795  },
{224  , 3791  },
{228  , 3786  },
{228  , 3785  },
{230  , 3782  },
{228  , 3779  },
{232  , 3775  },
{233  , 3772  },
{232  , 3770  },
{232  , 3765  },
{238  , 3763  },
{239  , 3760  },
{238  , 3758  },
{239  , 3753  },
{241  , 3748  },
{241  , 3744  },
{245  , 3742  },
{244  , 3738  },
{247  , 3733  },
{250  , 3724  },
{247  , 3717  },
{251  , 3712  },
{255  , 3706  },
{255  , 3699  },
{258  , 3694  },
{259  , 3688  },
{265  , 3684  },
{270  , 3680  },
{280  , 3678  },
{292  , 3674  },
{311  , 3669  },
{362  , 3658  },
{390  , 3642  },
{423  , 3615  },
{470  , 3578  },
{540  , 3532  },
{641  , 3477  },
{811  , 3406  }
#endif	
};
/* T2 25C*/
R_PROFILE_STRUCT r_profile_t2[] = {
#if 0	
	{107	  ,  4334},
	{107	  ,  4309},
	{114	  ,  4299},
	{112	  ,  4286},
	{108	  ,  4267},
	{111	  ,  4258},
	{111	  ,  4248},
	{117	  ,  4224},
	{110	  ,  4213},
	{116	  ,  4200},
	{114	  ,  4189},
	{109	  ,  4179},
	{110	  ,  4165},
	{112	  ,  4156},
	{108	  ,  4142},
	{107	  ,  4132},
	{107	  ,  4123},
	{108	  ,  4109},
	{114	  ,  4103},
	{110	  ,  4092},
	{117	  ,  4081},
	{117	  ,  4066},
	{120	  ,  4058},
	{122	  ,  4049},
	{118	  ,  4037},
	{120	  ,  4018},
	{118	  ,  4007},
	{117	  ,  3998},
	{120	  ,  3988},
	{119	  ,  3981},
	{122	  ,  3977},
	{121	  ,  3970},
	{119	  ,  3964},
	{119	  ,  3958},
	{122	  ,  3942},
	{124	  ,  3936},
	{126	  ,  3926},
	{138	  ,  3924},
	{122	  ,  3912},
	{129	  ,  3900},
	{123	  ,  3891},
	{119	  ,  3881},
	{114	  ,  3869},
	{112	  ,  3860},
	{109	  ,  3855},
	{109	  ,  3846},
	{108	  ,  3841},
	{106	  ,  3830},
	{105	  ,  3826},
	{108	  ,  3822},
	{107	  ,  3816},
	{110	  ,  3814},
	{112	  ,  3811},
	{108	  ,  3806},
	{111	  ,  3803},
	{113	  ,  3802},
	{113	  ,  3797},
	{111	  ,  3794},
	{112	  ,  3791},
	{109	  ,  3787},
	{111	  ,  3785},
	{107	  ,  3780},
	{108	  ,  3778},
	{111	  ,  3776},
	{114	  ,  3774},
	{116	  ,  3773},
	{113	  ,  3769},
	{113	  ,  3766},
	{114	  ,  3763},
	{112	  ,  3760},
	{116	  ,  3757},
	{116	  ,  3754},
	{112	  ,  3747},
	{114	  ,  3744},
	{114	  ,  3738},
	{116	  ,  3734},
	{109	  ,  3728},
	{107	  ,  3719},
	{112	  ,  3714},
	{109	  ,  3709},
	{114	  ,  3703},
	{112	  ,  3695},
	{113	  ,  3688},
	{114	  ,  3686},
	{118	  ,  3686},
	{120	  ,  3685},
	{121	  ,  3682},
	{124	  ,  3681},
	{126	  ,  3652},
	{127	  ,  3613},
	{128	  ,  3565},
	{138	  ,  3550},
	{150	  ,  3540},
	{216	  ,  3500}
#else
{109  ,4197 },
{109  ,4184 },
{111  ,4173 },
{110  ,4162 },
{107  ,4151 },
{106  ,4141 },
{111  ,4134 },
{106  ,4114 },
{108  ,4105 },
{109  ,4097 },
{112  ,4089 },
{110  ,4080 },
{110  ,4072 },
{109  ,4062 },
{110  ,4056 },
{111  ,4047 },
{112  ,4039 },
{116  ,4033 },
{114  ,4026 },
{115  ,4011 },
{114  ,4003 },
{117  ,3997 },
{118  ,3991 },
{117  ,3983 },
{121  ,3978 },
{121  ,3972 },
{119  ,3963 },
{122  ,3959 },
{124  ,3953 },
{126  ,3948 },
{128  ,3942 },
{127  ,3930 },
{130  ,3925 },
{129  ,3919 },
{129  ,3913 },
{130  ,3908 },
{132  ,3902 },
{129  ,3897 },
{127  ,3890 },
{126  ,3884 },
{127  ,3879 },
{126  ,3873 },
{124  ,3864 },
{123  ,3859 },
{117  ,3842 },
{117  ,3837 },
{116  ,3833 },
{115  ,3828 },
{113  ,3823 },
{111  ,3815 },
{112  ,3813 },
{111  ,3809 },
{110  ,3806 },
{110  ,3802 },
{112  ,3797 },
{113  ,3795 },
{110  ,3791 },
{110  ,3786 },
{110  ,3783 },
{109  ,3781 },
{113  ,3780 },
{113  ,3778 },
{113  ,3776 },
{114  ,3770 },
{114  ,3769 },
{113  ,3766 },
{113  ,3764 },
{115  ,3760 },
{115  ,3758 },
{115  ,3752 },
{112  ,3746 },
{111  ,3742 },
{111  ,3737 },
{113  ,3729 },
{116  ,3726 },
{115  ,3721 },
{117  ,3716 },
{116  ,3711 },
{119  ,3706 },
{119  ,3700 },
{117  ,3693 },
{116  ,3685 },
{114  ,3678 },
{114  ,3674 },
{116  ,3671 },
{146  ,3652 },
{169  ,3649 },
{174  ,3645 },
{196  ,3646 },
{133  ,3625 },
{133  ,3589 },
{134  ,3544 },
{142  ,3491 },
{150  ,3423 }
#endif	
};
/* T3 50C*/
R_PROFILE_STRUCT r_profile_t3[] = {
#if 0
	{93 	,  4325},
	{93 	,  4312},
	{92 	,  4297},
	{90 	,  4282},
	{90 	,  4269},
	{89 	,  4258},
	{89 	,  4245},
	{91 	,  4233},
	{90 	,  4222},
	{89 	,  4209},
	{90 	,  4198},
	{89 	,  4186},
	{92 	,  4176},
	{93 	,  4166},
	{92 	,  4153},
	{94 	,  4143},
	{94 	,  4132},
	{93 	,  4121},
	{92 	,  4109},
	{94 	,  4099},
	{92 	,  4087},
	{93 	,  4077},
	{93 	,  4066},
	{95 	,  4057},
	{94 	,  4047},
	{95 	,  4037},
	{95 	,  4027},
	{96 	,  4018},
	{95 	,  4008},
	{96 	,  3999},
	{98 	,  3990},
	{98 	,  3983},
	{98 	,  3974},
	{100	,  3967},
	{99 	,  3957},
	{101	,  3950},
	{102	,  3942},
	{103	,  3935},
	{102	,  3926},
	{103	,  3917},
	{102	,  3908},
	{99 	,  3896},
	{95 	,  3882},
	{92 	,  3871},
	{92 	,  3863},
	{91 	,  3855},
	{93 	,  3850},
	{92 	,  3843},
	{93 	,  3839},
	{91 	,  3833},
	{93 	,  3830},
	{91 	,  3824},
	{91 	,  3819},
	{91 	,  3815},
	{92 	,  3812},
	{93 	,  3807},
	{94 	,  3805},
	{94 	,  3800},
	{94 	,  3797},
	{96 	,  3795},
	{93 	,  3791},
	{97 	,  3789},
	{95 	,  3785},
	{95 	,  3783},
	{96 	,  3780},
	{98 	,  3778},
	{95 	,  3774},
	{94 	,  3768},
	{90 	,  3761},
	{93 	,  3755},
	{93 	,  3751},
	{92 	,  3746},
	{92 	,  3743},
	{96 	,  3740},
	{92 	,  3735},
	{94 	,  3732},
	{95 	,  3729},
	{97 	,  3724},
	{96 	,  3718},
	{93 	,  3709},
	{94 	,  3704},
	{96 	,  3698},
	{95 	,  3689},
	{92 	,  3680},
	{95 	,  3680},
	{95 	,  3679},
	{98 	,  3679},
	{99 	,  3676},
	{101	,  3673},
	{101	,  3663},
	{97 	,  3627},
	{97 	,  3577},
	{103	,  3510},
	{106    ,  3500}
#else
{96  , 4194},
{96  , 4181},
{98  , 4170},
{97  , 4160},
{96  , 4149},
{90  , 4138},
{90  , 4128},
{95  , 4121},
{93  , 4111},
{91  , 4093},
{93  , 4086},
{94  , 4077},
{94  , 4069},
{95  , 4060},
{97  , 4052},
{100 , 4045},
{98  , 4036},
{98  , 4030},
{98  , 4022},
{100 , 4015},
{94  , 4007},
{98  , 4000},
{93  , 3990},
{98  , 3986},
{98  , 3973},
{100 , 3966},
{102 , 3954},
{102 , 3948},
{101 , 3942},
{110 , 3938},
{103 , 3929},
{103 , 3919},
{105 , 3914},
{106 , 3909},
{103 , 3903},
{110 , 3900},
{109 , 3895},
{108 , 3888},
{107 , 3878},
{106 , 3871},
{100 , 3862},
{99  , 3854},
{106 , 3849},
{102 , 3841},
{101 , 3835},
{101 , 3831},
{98  , 3828},
{96  , 3822},
{95  , 3817},
{98  , 3814},
{101 , 3807},
{104 , 3806},
{99  , 3799},
{95  , 3794},
{102 , 3790},
{99  , 3787},
{103 , 3785},
{103 , 3782},
{103 , 3777},
{102 , 3774},
{111 , 3776},
{106 , 3772},
{101 , 3769},
{105 , 3767},
{107 , 3764},
{99  , 3755},
{100 , 3749},
{100 , 3745},
{101 , 3739},
{102 , 3737},
{105 , 3733},
{103 , 3729},
{103 , 3725},
{103 , 3720},
{99  , 3716},
{101 , 3713},
{102 , 3709},
{100 , 3699},
{102 , 3694},
{104 , 3682},
{101 , 3673},
{100 , 3664},
{98  , 3660},
{99  , 3658},
{107 , 3657},
{105 , 3653},
{106 , 3650},
{108 , 3648},
{114 , 3640},
{106 , 3617},
{105 , 3586},
{107 , 3545},
{110 , 3495},
{115 , 3437}

#endif	
};
/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3*/
R_PROFILE_STRUCT r_profile_temperature[] = {
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0},
		{0, 0}
};

/* ============================================================*/
/* function prototype*/
/* ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif	/*#ifndef _MTK_BATTERY_METER_TABLE_H*/

