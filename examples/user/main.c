// =============================================================================
// Copyright (C) EBOTS Inc.
// =============================================================================
// FILE NAME	: main.c
// DEPARTMENT	: 
// AUTHOR		: Hung Lam
// DATE			: Apr 03 2021
// =============================================================================
// DESCRIPTION	: ECAT Test for Maxon motordrive
//              : Motordrive is tested in PPM mode
// =============================================================================
// REVISION HISTORY	:
//	v1.0			: Initial version release
// =============================================================================

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <time.h>       /* clock_gettime() */
#include <sys/mman.h>   /* mlockall() */
#include <sched.h>      /* sched_setscheduler() */
#include <pthread.h>
// #include <rtdm/rtdm.h>

/****************************************************************************/

#include "ecrt.h"

/****************************************************************************/

/** Task period in ns. */
#define PERIOD_NS   (1000000)

#define MAX_SAFE_STACK (8 * 1024) /* The maximum stack size which is
                                     guranteed safe to access without
                                     faulting */

/****************************************************************************/

/* Constants */
#define NSEC_PER_SEC                        (1000000000)
#define FREQUENCY                           (NSEC_PER_SEC / PERIOD_NS)

#define ETHERCAT_STATUS_OP                  (0x08)
#define STATUS_SERVO_ENABLE_BIT             (0x04)
/****************************************************************************/

//master status
typedef enum  _SysWorkingStatus
{
    SYS_WORKING_POWER_ON,
    SYS_WORKING_SAFE_MODE,
    SYS_WORKING_OP_MODE,
    SYS_WORKING_LINK_DOWN,
    SYS_WORKING_IDLE_STATUS       //System is idle
}SysWorkingStatus;

typedef  struct  _GSysRunningParm
{
    SysWorkingStatus   m_gWorkStatus;
}GSysRunningParm;

GSysRunningParm    gSysRunning;

/****************************************************************************/
// Thread data
static int iothread_run; // 0: exit, 1: run
static int mdthread_run; // 0: exit, 1: run

static unsigned int cycle_us = 1000; /* 1 ms */

static pthread_t cyclic_thread;
/****************************************************************************/

static int ecstate = 0;

// EtherCAT
// Master
static ec_master_t      *master = NULL;
static ec_master_state_t master_state = {};

// domains
static ec_domain_t      *domain_io = NULL;
static ec_domain_state_t domain_io_state = {};

static ec_domain_t      *domainServoInput = NULL;
static ec_domain_state_t domainServoInput_state = {};
static ec_domain_t      *domainServoOutput = NULL;
static ec_domain_state_t domainServoOutput_state = {};

static ec_slave_config_t        *sc_abt_io = NULL;
static ec_slave_config_state_t  sc_abt_io_state = {};

static ec_slave_config_t        *sc_maxondrive;
static ec_slave_config_state_t  sc_maxondrive_state;

// process data
static uint8_t *domain_io_pd = NULL;
static uint8_t *domain_maxondrive_out_pd = NULL;
static uint8_t *domain_maxondrive_in_pd = NULL;

/****************************************************************************/

#define AbtIOSlavePos  0, 0
#define MaxEp4SlavePos 0, 1

// Vendor ID, Product ID
#define ABT_IOBOARD     0x0000079A, 0x00DEFEDE
#define MAXON_EPOS4     0x000000FB, 0x60500000

// user mem-map for ethercat slaves
static unsigned char    user_digou_u8; 
static unsigned char    user_digin_u8; 
static unsigned char    user_anain_u8[2]; 
static unsigned char    user_cnter_u8[2]; 

// offsets for PDO entries
// IO board
static unsigned int off_abt_out;
static unsigned int off_abt_in;
// Maxon servo drive
static unsigned int cntlwd_x6040;
static unsigned int tarpos_x607a;
static unsigned int modeop_x6060;
static unsigned int status_x6041;
static unsigned int actpos_x6064;
static unsigned int actvel_x606c;
static unsigned int acttor_x6077;
static unsigned int mopdis_x6061;
// servo driver homing mode
static unsigned int hommet_x6098;
static unsigned int spswit_x6099;
static unsigned int spzero_x6099;
static unsigned int homacc_x609a;
static unsigned int homoff_x30b1;
static unsigned int hompos_x30b0;

// process data
// IO board
const static ec_pdo_entry_reg_t domain_io_regs[] = {
    {AbtIOSlavePos, ABT_IOBOARD, 0x0005, 1, &off_abt_out, NULL},
    {AbtIOSlavePos, ABT_IOBOARD, 0x0006, 1, &off_abt_in, NULL},
    {}
};
// Maxon motordrive
const static ec_pdo_entry_reg_t domain_maxondrive_out_regs[] = {
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6040, 0x00, &cntlwd_x6040, NULL},           // Control word
    {MaxEp4SlavePos, MAXON_EPOS4, 0x607a, 0x00, &tarpos_x607a, NULL},           // Target position
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6060, 0x00, &modeop_x6060, NULL},			// 6060 mode selection
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6098, 0x00, &hommet_x6098, NULL},			// Homing_method mode selection
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6099, 0x01, &spswit_x6099, NULL},			// speed_during_search_for_switch
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6099, 0x02, &spzero_x6099, NULL},			// speed_during_search_for_zero
    {MaxEp4SlavePos, MAXON_EPOS4, 0x609a, 0x00, &homacc_x609a, NULL},			// homing_acceleration
    {MaxEp4SlavePos, MAXON_EPOS4, 0x30b1, 0x00, &homoff_x30b1, NULL},			// home_offset
    {MaxEp4SlavePos, MAXON_EPOS4, 0x30b0, 0x00, &hompos_x30b0, NULL},			// home_position
    {}
};
const static ec_pdo_entry_reg_t domain_maxondrive_in_regs[] = {
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6041, 0x00, &status_x6041, NULL},           // Status word
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6064, 0x00, &actpos_x6064, NULL},           // Actual position
    {MaxEp4SlavePos, MAXON_EPOS4, 0x606c, 0x00, &actvel_x606c, NULL},           // Actual Speed
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6077, 0x00, &acttor_x6077, NULL},           // Actual Torque
    {MaxEp4SlavePos, MAXON_EPOS4, 0x6061, 0x00, &mopdis_x6061, NULL},           // Mode of operation display
    {}
};

static int cur_status;
static int cur_mode;

static unsigned int counter = 0;
static unsigned int blink = 0;
static unsigned int stickount = 0; 
/*****************************************************************************/

/* Master 0, Slave 0, "EasyCAT 32+32 rev 1"
 * Vendor ID:       0x0000079a
 * Product code:    0x00defede
 * Revision number: 0x00005a01
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x0005, 0x01, 8}, /* Byte0 */
    {0x0005, 0x02, 8}, /* Byte1 */
    {0x0005, 0x03, 8}, /* Byte2 */
    {0x0005, 0x04, 8}, /* Byte3 */
    {0x0005, 0x05, 8}, /* Byte4 */
    {0x0005, 0x06, 8}, /* Byte5 */
    {0x0005, 0x07, 8}, /* Byte6 */
    {0x0005, 0x08, 8}, /* Byte7 */
    {0x0005, 0x09, 8}, /* Byte8 */
    {0x0005, 0x0a, 8}, /* Byte9 */
    {0x0005, 0x0b, 8}, /* Byte10 */
    {0x0005, 0x0c, 8}, /* Byte11 */
    {0x0005, 0x0d, 8}, /* Byte12 */
    {0x0005, 0x0e, 8}, /* Byte13 */
    {0x0005, 0x0f, 8}, /* Byte14 */
    {0x0005, 0x10, 8}, /* Byte15 */
    {0x0005, 0x11, 8}, /* Byte16 */
    {0x0005, 0x12, 8}, /* Byte17 */
    {0x0005, 0x13, 8}, /* Byte18 */
    {0x0005, 0x14, 8}, /* Byte19 */
    {0x0005, 0x15, 8}, /* Byte20 */
    {0x0005, 0x16, 8}, /* Byte21 */
    {0x0005, 0x17, 8}, /* Byte22 */
    {0x0005, 0x18, 8}, /* Byte23 */
    {0x0005, 0x19, 8}, /* Byte24 */
    {0x0005, 0x1a, 8}, /* Byte25 */
    {0x0005, 0x1b, 8}, /* Byte26 */
    {0x0005, 0x1c, 8}, /* Byte27 */
    {0x0005, 0x1d, 8}, /* Byte28 */
    {0x0005, 0x1e, 8}, /* Byte29 */
    {0x0005, 0x1f, 8}, /* Byte30 */
    {0x0005, 0x20, 8}, /* Byte31 */
    {0x0006, 0x01, 8}, /* Byte0 */
    {0x0006, 0x02, 8}, /* Byte1 */
    {0x0006, 0x03, 8}, /* Byte2 */
    {0x0006, 0x04, 8}, /* Byte3 */
    {0x0006, 0x05, 8}, /* Byte4 */
    {0x0006, 0x06, 8}, /* Byte5 */
    {0x0006, 0x07, 8}, /* Byte6 */
    {0x0006, 0x08, 8}, /* Byte7 */
    {0x0006, 0x09, 8}, /* Byte8 */
    {0x0006, 0x0a, 8}, /* Byte9 */
    {0x0006, 0x0b, 8}, /* Byte10 */
    {0x0006, 0x0c, 8}, /* Byte11 */
    {0x0006, 0x0d, 8}, /* Byte12 */
    {0x0006, 0x0e, 8}, /* Byte13 */
    {0x0006, 0x0f, 8}, /* Byte14 */
    {0x0006, 0x10, 8}, /* Byte15 */
    {0x0006, 0x11, 8}, /* Byte16 */
    {0x0006, 0x12, 8}, /* Byte17 */
    {0x0006, 0x13, 8}, /* Byte18 */
    {0x0006, 0x14, 8}, /* Byte19 */
    {0x0006, 0x15, 8}, /* Byte20 */
    {0x0006, 0x16, 8}, /* Byte21 */
    {0x0006, 0x17, 8}, /* Byte22 */
    {0x0006, 0x18, 8}, /* Byte23 */
    {0x0006, 0x19, 8}, /* Byte24 */
    {0x0006, 0x1a, 8}, /* Byte25 */
    {0x0006, 0x1b, 8}, /* Byte26 */
    {0x0006, 0x1c, 8}, /* Byte27 */
    {0x0006, 0x1d, 8}, /* Byte28 */
    {0x0006, 0x1e, 8}, /* Byte29 */
    {0x0006, 0x1f, 8}, /* Byte30 */
    {0x0006, 0x20, 8}, /* Byte31 */
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 32, slave_0_pdo_entries + 0}, /* Outputs */
    {0x1a00, 32, slave_0_pdo_entries + 32}, /* Inputs */
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {1, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/* Master 0, Slave 1, "EPOS4"
 * Vendor ID:       0x000000fb
 * Product code:    0x60500000
 * Revision number: 0x01610000
 */

ec_pdo_entry_info_t maxon_pdo_entries[] = {
    //CoE CSP Mode (RX)
    {0x6040, 0x00, 16},     /* Controlword */
    {0x607a, 0x00, 32},     /* Target Position */
    {0x6060, 0x00, 8},      /*modes of operation*/
    {0x6098, 0x00, 8},      /*Homing method*/
    {0x6099, 0x01, 32},     /*Speed during search for switch*/
    {0x6099, 0x02, 32},     /*Speed during search for zero*/
    {0x609a, 0x00, 32},     /*homing_acceleration*/
    {0x30b1, 0x00, 32},     /*home_offset*/
    {0x30b0, 0x00, 32},     /*home_pos*/
    //CoE CSP Mode (TX)
    {0x6041, 0x00, 16},     /* Statusword */
    {0x6064, 0x00, 32},     /* Position actual value */
    {0x606c, 0x00, 32},     /* Speed actual value */
    {0x6077, 0x00, 16},     /* Torque actual value */
    {0x6061, 0x00, 8},      /*modes of operation display*/
};

ec_pdo_info_t maxon_pdos[] = {
    {0x1600, 9, maxon_pdo_entries + 0},     /* CoE CSP Mode (RX) */
    {0x1a00, 5, maxon_pdo_entries + 9},     /* CoE CSP Mode (TX) */
};

ec_sync_info_t maxon_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, maxon_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, maxon_pdos + 1, EC_WD_DISABLE},
    {0xff}
};

/*****************************************************************************/
// TEST MENU
typedef enum {
    // Reserved - 0
    MENU_ABT_DIG_RD = 1, 
    MENU_ABT_DIG_WR, 
    MENU_ABT_ANA_RD, 
    MENU_MAXONDRIVE_INFO_RD, 
    MENU_MAXONDRIVE_MOTI_WR, 
    MENU_QUIT = 99
} MENU_ID;

static int user_sel = 0; 
static int user_quit = 0; 

void UI_ShowMenu(void){
    // Menu display
    printf(" __________________________________________________________ \r\n");
	printf("|==========================================================|\r\n");
	printf("| [%d]: ABT ECAT IOBOARD - DIGITAL READ                     |\r\n", MENU_ABT_DIG_RD);
	printf("| [%d]: ABT ECAT IOBOARD - DIGITAL WRITE                    |\r\n", MENU_ABT_DIG_WR);
	printf("| [%d]: ABT ECAT IOBOARD - ANALOG  READ                     |\r\n", MENU_ABT_ANA_RD);
    printf("|----------------------------------------------------------|\r\n");
	printf("| [%d]: MAXON ECAT DRIVE - STATUS  READ                     |\r\n", MENU_MAXONDRIVE_INFO_RD);
	printf("| [%d]: MAXON ECAT DRIVE - MOTION  WRITE                    |\r\n", MENU_MAXONDRIVE_MOTI_WR);
    printf("|----------------------------------------------------------|\r\n");
	printf("| [%d]: Quit                                               |\r\n", MENU_QUIT);
    printf("|==========================================================|\r\n");
    // User Selection
	printf(" Plesae input your selection:");
}

int UI_UserSelect(void){
	int nSel;
	scanf("%d", &nSel);
	return nSel;
}

/*****************************************************************************/

void rt_check_domain_io_state(void)
{
    ec_domain_state_t ds_io;
    // domain io
    ecrt_domain_state(domain_io, &ds_io);
    if (ds_io.working_counter != domain_io_state.working_counter) {
        // printf("domain_io: WC %u.\n", ds_io.working_counter);
    }
    if (ds_io.wc_state != domain_io_state.wc_state) {
        // printf("domain_io: State %u.\n", ds_io.wc_state);
    }
    domain_io_state = ds_io;
        
}

void rt_check_domain_maxondrive_state(void)
{
    ec_domain_state_t ds_maxon_in;
    ec_domain_state_t ds_maxon_out;
    // domain servo input
    ecrt_domain_state(domainServoInput, &ds_maxon_in);
    if (ds_maxon_in.working_counter != domainServoInput_state.working_counter) {
        // printf("ds_maxon_in: WC %u.\n", ds_maxon_in.working_counter);
    }
    if (ds_maxon_in.wc_state != domainServoInput_state.wc_state) {
        // printf("ds_maxon_in: State %u.\n", ds_maxon_in.wc_state);
    }
    domainServoInput_state = ds_maxon_in;   
    // domain servo output
    ecrt_domain_state(domainServoOutput, &ds_maxon_out);
    if (ds_maxon_out.working_counter != domainServoOutput_state.working_counter) {
        // printf("ds_maxon_out: WC %u.\n", ds_maxon_out.working_counter);
    }
    if (ds_maxon_out.wc_state != domainServoOutput_state.wc_state) {
        // printf("ds_maxon_out: State %u.\n", ds_maxon_out.wc_state);
    }
    domainServoOutput_state = ds_maxon_out;      
}
/*****************************************************************************/

void check_master_state(void)
{
    ec_master_state_t ms;

    ecrt_master_state(master, &ms);

    if (ms.slaves_responding != master_state.slaves_responding) {
        printf("%u slave(s).\n", ms.slaves_responding);
    }
    if (ms.al_states != master_state.al_states) {
        printf("AL states: 0x%02X.\n", ms.al_states);
    }
    if (ms.link_up != master_state.link_up) {
        printf("Link is %s.\n", ms.link_up ? "up" : "down");
    }

    master_state = ms;
}

/*****************************************************************************/

void check_io_slave_config_states(void)
{
    ec_slave_config_state_t s_io;

    // IO board
    ecrt_slave_config_state(sc_abt_io, &s_io);

    if (s_io.al_state != sc_abt_io_state.al_state) {
        printf("Abt IO board: State 0x%02X.\n", s_io.al_state);
    }
    if (s_io.online != sc_abt_io_state.online) {
        printf("Abt IO board: %s.\n", s_io.online ? "online" : "offline");
    }
    if (s_io.operational != sc_abt_io_state.operational) {
        printf("Abt IO board: %soperational.\n", s_io.operational ? "" : "Not ");
    }
    sc_abt_io_state = s_io;
  
}

void check_maxondrive_slave_config_states(void)
{
    ec_slave_config_state_t s_maxon;

    // Maxon motordrive
    ecrt_slave_config_state(sc_abt_io, &s_maxon);

    if (s_maxon.al_state != sc_maxondrive_state.al_state) {
        printf("Maxon Motordrive: State 0x%02X.\n", s_maxon.al_state);
    }
    if (s_maxon.online != sc_maxondrive_state.online) {
        printf("Maxon Motordrive: %s.\n", s_maxon.online ? "online" : "offline");
    }
    if (s_maxon.operational != sc_maxondrive_state.operational) {
        printf("Maxon Motordrive: %soperational.\n", s_maxon.operational ? "" : "Not ");
    }
    sc_maxondrive_state = s_maxon;    
}

/*****************************************************************************/

void io_cyclic_task()
{
    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domain_io);

    // check process data state
    rt_check_domain_io_state();

    if (counter) {
        counter--;
    } else { // do this at 1 Hz
        counter = FREQUENCY;

        // check for master state (optional)
        check_master_state();

        // check for slave configuration state(s) (optional)
        check_io_slave_config_states();
    }

#if 1
    user_anain_u8[0] = EC_READ_U8(domain_io_pd + off_abt_in + 0);
    user_anain_u8[1] = EC_READ_U8(domain_io_pd + off_abt_in + 1);
    user_cnter_u8[0] = EC_READ_U8(domain_io_pd + off_abt_in + 2);
    user_cnter_u8[1] = EC_READ_U8(domain_io_pd + off_abt_in + 3);
    user_digin_u8    = EC_READ_U8(domain_io_pd + off_abt_in + 6);

    // write process data
    EC_WRITE_U8(domain_io_pd + off_abt_out, user_digou_u8);
#endif

    // send process data
    ecrt_domain_queue(domain_io);
    ecrt_master_send(master);
}

void maxondrive_cyclic_task()
{
    static int curpos = 0;
    static int curvel = 0;
    //Just boot (need to wait for other operations to complete), return to wait for the next cycle
    if(gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON)
        return ;

    static int cycle_counter = 0;
    cycle_counter++;
    if(cycle_counter >= 120*1000){      // Max cycle counter
        cycle_counter = 0;
	    // run = 0;
    }

    // receive process data
    ecrt_master_receive(master);
    ecrt_domain_process(domainServoInput);
    ecrt_domain_process(domainServoOutput);

    // check process data state
    rt_check_domain_maxondrive_state();

    if (!(cycle_counter % 500)) {
        check_master_state();
        check_maxondrive_slave_config_states();
    }

    //State machine operation
    switch (gSysRunning.m_gWorkStatus)
    {
        case SYS_WORKING_SAFE_MODE:
            //Check whether the master station is in OP mode, if not, adjust to OP mode
            check_master_state();
            check_maxondrive_slave_config_states();
            if((master_state.al_states & ETHERCAT_STATUS_OP))
            {
                if(sc_maxondrive_state.al_state != ETHERCAT_STATUS_OP){
                    break ;
                }

                ecstate = 0;
                gSysRunning.m_gWorkStatus = SYS_WORKING_OP_MODE;
                printf("Maxon motor drive next-ecmode: SYS_WORKING_OP_MODE\n");
                
            }
        break; 

        case SYS_WORKING_OP_MODE: 
            ecstate++;
            //Enable servo
            if(ecstate <= 1000){
                switch (ecstate){
                case 1:
                    EC_WRITE_U8(domain_maxondrive_out_pd + modeop_x6060, 1);        // PPM mode
                    break;
                case 200:
                    EC_WRITE_U16(domain_maxondrive_out_pd + cntlwd_x6040, 0x08);    //Error reset   
                    break;
                case 300:                   
                    curpos  = EC_READ_S32(domain_maxondrive_in_pd + actpos_x6064);                       
                    printf("x@rtITP >>> Axis current position = %d\n", curpos);
                    break;
                case 400:
                    EC_WRITE_U16(domain_maxondrive_out_pd + cntlwd_x6040, 0x06);
                    break;
                case 500:
                    EC_WRITE_U16(domain_maxondrive_out_pd + cntlwd_x6040, 0x0F);
                    break;
                }            
            }
            // else {
            //     printf("enable servo success!\n");
            //     cur_mode= EC_READ_U8(domain_maxondrive_in_pd + mopdis_x6061);
            //     printf("modes_of_operation_display 0x6061 = %d\n",cur_mode);

            //     cur_status = EC_READ_U16(domain_maxondrive_in_pd + status_x6041);
            //     printf("status_world 0x6041 = %d\n",cur_status);

            //     if((EC_READ_U16(domain_maxondrive_in_pd + status_x6041) & (STATUS_SERVO_ENABLE_BIT)) == 0){
            //         break ;
            //     }

            //     ecstate = 0;
            //     gSysRunning.m_gWorkStatus = SYS_WORKING_IDLE_STATUS;
            //     printf("Maxon motor drive next-ecmode: SYS_WORKING_IDLE_STATUS\n");
            // }
        break; 

        default: 
            if (!(cycle_counter % 2000)) {
                // printout current position/velocity/status of maxon motordrive
                cur_status = EC_READ_U16(domain_maxondrive_in_pd + status_x6041);
                printf("cur_status: %d\t", cur_status);
                curpos = EC_READ_S32(domain_maxondrive_in_pd + actpos_x6064);
                printf("curpos = %d\t",curpos);
                curvel = EC_READ_S32(domain_maxondrive_in_pd + actvel_x606c);
                printf("curvel = %d\t",curvel);
            }
        break; 
    }
    // send process data
    ecrt_domain_queue(domainServoInput);
    ecrt_domain_queue(domainServoOutput);
    ecrt_master_send(master);    
}
/****************************************************************************/

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];

    memset(dummy, 0, MAX_SAFE_STACK);
}

/****************************************************************************/

void ReleaseMaster()
{
    if(master)
    {
        printf("Ethercat Test End of Program, release master\n");
        ecrt_release_master(master);
        master = NULL;
    }
}

/****************************************************************************/
int ConfigPDO(){
    // create domain_io
    domain_io = ecrt_master_create_domain(master);
    if (!domain_io) {
        return -1;
    }

    // create domainServoInput
    domainServoInput = ecrt_master_create_domain(master);
    if (!domainServoInput) {
        return -1;
    }

    // create domainServoOutput
    domainServoOutput = ecrt_master_create_domain(master);
    if (!domainServoOutput) {
        return -1;
    }
    printf("ECAT Creating slave configurations...\n");
    // Obtains a slave configuration - AB&T Ethercat Slave IO board
    if (!(sc_abt_io = ecrt_master_slave_config(
                    master, AbtIOSlavePos, ABT_IOBOARD))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    // Obtains a slave configuration - AB&T Ethercat Slave IO board
    if (!(sc_maxondrive = ecrt_master_slave_config(
                    master, MaxEp4SlavePos, MAXON_EPOS4))) {
        fprintf(stderr, "Failed to get slave configuration.\n");
        return -1;
    }

    printf("ECAT Configuring PDOs...\n");
    if (ecrt_slave_config_pdos(sc_abt_io, EC_END, slave_0_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }

    if (ecrt_domain_reg_pdo_entry_list(domain_io, domain_io_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }

    if (ecrt_slave_config_pdos(sc_maxondrive, EC_END, maxon_syncs)) {
        fprintf(stderr, "Failed to configure PDOs.\n");
        return -1;
    }
    if (ecrt_domain_reg_pdo_entry_list(domainServoInput, domain_maxondrive_in_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
    if (ecrt_domain_reg_pdo_entry_list(domainServoOutput, domain_maxondrive_out_regs)) {
        fprintf(stderr, "PDO entry registration failed!\n");
        return -1;
    }
    // printf("value of off_abt_out: %d\n", off_abt_out); 
    // printf("value of off_abt_in: %d\n", off_abt_in); 
    return 0; 
}

/****************************************************************************/

int ActivateMaster(void){
    printf("ECAT Requesting master...\n");
    master = ecrt_request_master(0);
    if (!master) {
        return -1;
    }

    ConfigPDO();
   
    /********************/
    printf("ECAT Activating master...\n");
    if (ecrt_master_activate(master)) {
        return -1;
    }
    /********************/

    if (!(domain_io_pd = ecrt_domain_data(domain_io))) {
        return -1;
    }
    if (!(domain_maxondrive_out_pd = ecrt_domain_data(domainServoInput))) {
        return -1;
    }
    if (!(domain_maxondrive_in_pd = ecrt_domain_data(domainServoOutput))) {
        return -1;
    }    
    return 0; 
}

/****************************************************************************/
//============================ TEST SUB-ROUTINES ===========================//

void ABT_DIG_RD(){
    printf("Digital Inputs: %u \r\n", user_digin_u8);
}

void ABT_DIG_WR(){
    int Mask;

    printf("Please enter digital value: ");
	scanf("%d", &Mask);

    // Read/Write Data
    user_digou_u8 = (Mask > 255)? 255: ((Mask < 0)? 0: Mask);
}

void ABT_ANA_RD(){
    // Read/Write Data
    printf("Analog In0: %u - Analog In1: %u - ConterUp.High: %u - ConterUp.Low: %u \r\n",
        user_anain_u8[0], user_anain_u8[1], user_cnter_u8[0], user_cnter_u8[1]);
}

void MAXONDRIVE_INFO_RD(){

}

void MAXONDRIVE_MOTI_WR(){

}

/****************************************************************************/
//========================= IO THREAD PROGRAM ==============================//

void *IOThread(void *arg){
    struct timespec next_period;
    clock_gettime(CLOCK_REALTIME, &next_period);

    while (iothread_run){
        next_period.tv_nsec += cycle_us * 1000;
        while (next_period.tv_nsec >= NSEC_PER_SEC) {
			next_period.tv_nsec -= NSEC_PER_SEC;
			next_period.tv_sec++;
		}
        clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &next_period, NULL);
        io_cyclic_task();
    }
    printf(" IO Thread termniated! \r\n"); 

    return NULL; 
}

/****************************************************************************/
//==================== MOTOR DRIVE THREAD PROGRAM ==========================//

void MDThread(void){
    while (mdthread_run){

    }
    printf(" Motor Drive Thread termniated! \r\n"); 
}

/****************************************************************************
 * Signal handler
 ***************************************************************************/

void signal_handler(int sig)
{
    iothread_run = 0;
    mdthread_run = 0; 
}

/****************************************************************************/
//============================ MAIN PROGRAM ================================//

int main(int argc, char **argv)
{
    // ec_slave_config_t *sc;
    struct timespec wakeup_time;
    int ret = 0;
    int count = 100000; 
    iothread_run = 1;
    mdthread_run = 1; 

    // rt_print_auto_init(1);
    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);
    mlockall(MCL_CURRENT | MCL_FUTURE);

    gSysRunning.m_gWorkStatus = SYS_WORKING_POWER_ON;
    if(gSysRunning.m_gWorkStatus == SYS_WORKING_POWER_ON){
        ActivateMaster(); 
        ecstate = 0;
        gSysRunning.m_gWorkStatus = SYS_WORKING_SAFE_MODE;
        printf("Maxon motor drive next-ecmode: SYS_WORKING_SAFE_MODE\n");
    }


    /* Create cyclic RT-thread */
    struct sched_param param = { .sched_priority = 82 };
    pthread_attr_t thattr;
    pthread_attr_init(&thattr);
    pthread_attr_setdetachstate(&thattr, PTHREAD_CREATE_JOINABLE);
    pthread_attr_setinheritsched(&thattr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&thattr, SCHED_FIFO);
    pthread_setschedparam(cyclic_thread, SCHED_FIFO, &param);

    ret = pthread_create(&cyclic_thread, &thattr, &IOThread, NULL);
    if (ret) {
        fprintf(stderr, "%s: pthread_create cyclic task failed\n",
                strerror(-ret));
		return 1;
    }

    // User Interface
    while (user_quit == 0) {
        sched_yield();
		UI_ShowMenu();
		user_sel = UI_UserSelect();
		switch (user_sel) {
			case MENU_ABT_DIG_RD:
				ABT_DIG_RD(); 
				break;
			case MENU_ABT_DIG_WR:
				ABT_DIG_WR(); 
				break;
			case MENU_ABT_ANA_RD:
				ABT_ANA_RD(); 
				break;
			case MENU_MAXONDRIVE_INFO_RD:
				MAXONDRIVE_INFO_RD(); 
				break;
			case MENU_MAXONDRIVE_MOTI_WR:
				MAXONDRIVE_MOTI_WR(); 
				break;
			
			case MENU_QUIT:
				user_quit = 1;
                iothread_run = 0;
                mdthread_run = 0; 
				printf("ECAT Test Software Stopped - Bye!\r\n");
				break;
			default:
				printf("[Warning] User Interface Invalid selection\r\n");
		}// switch
	}// while

    pthread_join(cyclic_thread, NULL);
    // Stop ECAT Test Software - Release Master
    ReleaseMaster();
    return ret;
}

/****************************************************************************/
