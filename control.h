#define STPR_STP 0
#define STPR_ON  1
#define OXY_SOLE_CYL_OFF 2
#define OXY_SOLE_CYL_ONN  3
#define OXY_SOLE_HOS_O2_OFF 4
#define OXY_SOLE_HOS_O2_ONN  5
#define INH_SOLE_OFF 6
#define INH_SOLE_ONN  7
#define EXH_SOLE_OFF 8
#define EXH_SOLE_ONN  9
#define PK_PR_REL_OFF 10
#define PK_PR_REL_ONN  11
#define SET_TID_VOL  12
#define SET_BPM  13
#define SET_PK_PR 14
#define SET_FIO2  15
#define SET_IE_RATIO 16
#define SL_EN_PARM_EDT  17
#define SL_COM_PARM_EDT 18
#define INIT_MAST  19
#define INIT_STPR_MOD 20
#define INIT_VALV_BLK  21
static const String commands[] =
{ "$VMST0000&",
  "$VMST0001&",
  "$VMO20100&",
  "$VMO20101&",
  "$VMO20200&",
  "$VMO20201&",
  "$VMSV0100&",
  "$VMSV0101&",
  "$VMSV0200&",
  "$VMSV0201&",
  "$VMSV0300&",
  "$VMSV0301&",
  "$VMP1xxxx&",
  "$VMP2xxxx&",
  "$VMP3xxxx&",
  "$VMP4xxxx&",
  "$VMP5xxxx&",
  "$VMPP0000&",
  "$VMPP1111&",
  "$VMIN0000&",
  "$VMIN0001&",
  "$VMIN0002&"
};
