/**
 * @file    linux-amd_energy.c
 * @author  Niccolo Tosato; based on Vince Weaver work.
 *
 * @ingroup papi_components
 *
 * @brief amd_energy component
 *
 *  This component enables RAPL (Running Average Power Level)
 *  energy measurements on AMD epyc processors.
 *
 *  To work, either msr_safe kernel module from LLNL
 *  (https://github.com/scalability-llnl/msr-safe), or
 *  the x86 generic MSR driver must be installed
 *    (CONFIG_X86_MSR) and the /dev/cpu/?/<msr_safe | msr> files must have read permissions
 */

#include <stdio.h>
#include <unistd.h>
#include <dirent.h>
#include <fcntl.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>

/* Headers required by PAPI */
#include "papi.h"
#include "papi_internal.h"
#include "papi_vector.h"
#include "papi_memory.h"

// The following macro follows if a string function has an error. It should 
// never happen; but it is necessary to prevent compiler warnings. We print 
// something just in case there is programmer error in invoking the function.
#define HANDLE_STRING_ERROR {fprintf(stderr,"%s:%i unexpected string function error.\n",__FILE__,__LINE__); exit(-1);}


/***************/
/* AMD Support */
/***************/
//questi sono corretti
#define MSR_AMD_RAPL_POWER_UNIT                 0xc0010299
#define MSR_AMD_PKG_ENERGY_STATUS               0xc001029B
#define MSR_AMD_PP0_ENERGY_STATUS               0xc001029A

#define POWER_UNIT_OFFSET          0
#define POWER_UNIT_MASK         0x0f

#define ENERGY_UNIT_OFFSET      0x08
#define ENERGY_UNIT_MASK        0x1f

typedef struct _energy_register
{
	unsigned int selector;
} _energy_register_t;

// still keep this
typedef struct _energy_native_event_entry
{
  char name[PAPI_MAX_STR_LEN];
  char units[PAPI_MIN_STR_LEN];
  char description[PAPI_MAX_STR_LEN];
  int fd_offset;
  int msr;
  int type;
  int return_type;
  _energy_register_t resources;
} _energy_native_event_entry_t;

//keep this
typedef struct _energy_reg_alloc
{
	_energy_register_t ra_bits;
} _energy_reg_alloc_t;


/* actually 32?  But setting this to be safe? */
/* since AMD has per-core counter, we need at least one counter per core + one per socket */
/*   this end up in at least 130*2 counters */
#define AMD_ENERGY_MAX_COUNTERS 260


typedef struct _energy_control_state
{
  int being_measured[AMD_ENERGY_MAX_COUNTERS];
  long long count[AMD_ENERGY_MAX_COUNTERS];
  int need_difference[AMD_ENERGY_MAX_COUNTERS];
  long long lastupdate;
} _energy_control_state_t;

// The _ENERGY_ counters should return a monotonically increasing
// value from the _start point, but the hardware only returns a
// uint32_t that may wrap. We keep a start_value which is reset at
// _start and every read, handle overflows of the uint32_t, and
// accumulate a uint64_t which we return.
// We must check which amd has 64bit registry, the wrap could never happen in certain models

typedef struct _energy_context
{
  long long start_value[AMD_ENERGY_MAX_COUNTERS];
  long long accumulated_value[AMD_ENERGY_MAX_COUNTERS];
  _energy_control_state_t state;
} _energy_context_t;


papi_vector_t _amd_energy_vector;

struct fd_array_t {
  int fd;
  int open;
};

static _energy_native_event_entry_t * energy_native_events=NULL;
static int num_events		= 0;
struct fd_array_t *fd_array=NULL;
static int num_packages=0,num_cpus=0;

int power_divisor;

int cpu_energy_divisor;
unsigned int msr_rapl_power_unit;

#define PACKAGE_ENERGY      	0
#define PACKAGE_ENERGY_CNT      5

/***************************************************************************/
/******  BEGIN FUNCTIONS  USED INTERNALLY SPECIFIC TO THIS COMPONENT *******/
/***************************************************************************/

static long long read_msr(int fd, unsigned int which) {
  //fd e' il file descriptor del registro, which e' l'address del registro
	uint64_t data;

	if ( fd<0 || pread(fd, &data, sizeof data, which) != sizeof data ) {
		perror("rdmsr:pread");
		fprintf(stderr,"rdmsr error, msr %x\n",which);
		exit(127);
	}

	return (long long)data;
}

static int open_fd(int offset) {
  //apre il FS per i MSR; offset e' il numero della CPU !!!!!
  // dovremmo essere ok con MSRSAFE, salva su un fd_array
  int fd=-1;
  char filename[BUFSIZ];

  if (fd_array[offset].open==0) {
	  sprintf(filename,"/dev/cpu/%d/msr_safe",offset);
      fd = open(filename, O_RDONLY);
	  if (fd<0) {
		  sprintf(filename,"/dev/cpu/%d/msr",offset);
          fd = open(filename, O_RDONLY);
	  }
	  if (fd>=0) {
		  fd_array[offset].fd=fd;
	      fd_array[offset].open=1;
      } 
  }
  else {
    fd=fd_array[offset].fd;
  }

  return fd;
}

//legge proprio, usa read_msr
static long long read_energy_value(int index) {

   int fd;

   fd=open_fd(energy_native_events[index].fd_offset);
   return read_msr(fd,energy_native_events[index].msr);

}

//prende la lettura da msr, la converte usando i vari divisori etc cazzare varie, sta cosa si ridurra a pochissime righe ! 
static long long convert_energy_readings(int index, long long value) {

   union {
      long long ll;
      double fp;
   } return_val;

   return_val.ll = value; /* default case: return raw input value */

   if (energy_native_events[index].type==PACKAGE_ENERGY) {
      return_val.ll = (long long)(((double)value/cpu_energy_divisor)*1e9);
   }

   return return_val.ll;
}


//sta cosa secondo me va cambiata e anche molto !!!!!!
static int
 get_kernel_nr_cpus(void)
{
  FILE *fff;
  int num_read, nr_cpus = 1;
  fff=fopen("/sys/devices/system/cpu/kernel_max","r");
  if (fff==NULL) return nr_cpus;
  num_read=fscanf(fff,"%d",&nr_cpus);
  fclose(fff);
  if (num_read==1) {
    nr_cpus++;
  } else {
    nr_cpus = 1;
  }
  return nr_cpus;
}

/************************* PAPI Functions **********************************/


/*
 * This is called whenever a thread is initialized
 */
static int
_amd_energy_init_thread( hwd_context_t *ctx )
{
  ( void ) ctx;

  return PAPI_OK;
}



/*
 * Called when PAPI process is initialized (i.e. PAPI_library_init)
 */
//inizializza il component, e' una palla sta roba qui 
static int
_amd_energy_init_component( int cidx )
{
    int retval = PAPI_OK;
    //boh
     int i,j,k,fd;
     FILE *fff;
     char filename[BUFSIZ];
     long unsigned int strErr;
     char *strCpy;
     // sto coso controlla se e' available il tutto
     /* int package_avail, dram_avail, pp0_avail, pp1_avail, psys_avail; */
     /* int different_units; */

     long long result;
     int package;
     // interessante
     const PAPI_hw_info_t *hw_info;
     // vediamo come funziona ! cpus = socket oppure core ? NUN SE SA ! 
     int nr_cpus = get_kernel_nr_cpus();
     //salva quanti packages
     int packages[nr_cpus];
     //non si capisce in realta che senso avrebbe sta cosa 
     int cpu_to_use[nr_cpus];
     
     unsigned int msr_pkg_energy_status,msr_pp0_energy_status;


     /* Fill with sentinel values */
     for (i=0; i<nr_cpus; ++i) {
       packages[i] = -1;
       cpu_to_use[i] = -1;
     }

     
     /* check if supported processor */
     hw_info=&(_papi_hwi_system_info.hw_info);

     /* Ugh can't use PAPI_get_hardware_info() if
	PAPI library not done initializing yet */
     //facciamo solo amd EPYC family sopra 17h etc cazzi mazzi
     switch(hw_info->vendor) {
		/* case PAPI_VENDOR_INTEL: */
		case PAPI_VENDOR_AMD:
			break;
		default:
			strCpy=strncpy(_amd_energy_vector.cmp_info.disabled_reason,
			"Not a supported processor",PAPI_MAX_STR_LEN);
			_amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
         if (strCpy == NULL) HANDLE_STRING_ERROR;
         retval = PAPI_ENOSUPP;
         goto fn_fail;
	}


     
     if (hw_info->vendor==PAPI_VENDOR_AMD) {
       //qui possiamo pensare che sta roba vada tenuta
       msr_rapl_power_unit=MSR_AMD_RAPL_POWER_UNIT;
       msr_pkg_energy_status=MSR_AMD_PKG_ENERGY_STATUS;
       msr_pp0_energy_status=MSR_AMD_PP0_ENERGY_STATUS;
       if (hw_info->cpuid_family!=0x17) {
	 /* Not a family 17h machine */
	 strCpy=strncpy(_amd_energy_vector.cmp_info.disabled_reason,
			"CPU family not supported",PAPI_MAX_STR_LEN);
	 _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
         if (strCpy == NULL) HANDLE_STRING_ERROR;
         retval = PAPI_ENOIMPL;
         goto fn_fail;
       }

     }

       
     /* Detect how many packages */
     // Some code below may be flagged by Coverity due to uninitialized array
     // entries of cpu_to_use[]. This is not a bug; the 'filename' listed below
     // will have 'cpu0', 'cpu1', sequentially on up to the maximum.  Coverity
     // cannot know that, so its code analysis allows the possibility that the
     // cpu_to_use[] array is only partially filled in. [Tony C. 11-27-19].

     j=0;
     while(1) {
       int num_read;
       
       strErr=snprintf(filename, BUFSIZ, 
	       "/sys/devices/system/cpu/cpu%d/topology/physical_package_id",j);
       filename[BUFSIZ-1]=0;
       if (strErr > BUFSIZ) HANDLE_STRING_ERROR;
       fff=fopen(filename,"r");
       if (fff==NULL) break;
       //leggo in che package sta il j-esimo  core
       num_read=fscanf(fff,"%d",&package);
       fclose(fff);
       if (num_read!=1) {
    		 strCpy=strcpy(_amd_energy_vector.cmp_info.disabled_reason, "Error reading file: ");
          if (strCpy == NULL) HANDLE_STRING_ERROR;
    		 strCpy=strncat(_amd_energy_vector.cmp_info.disabled_reason, filename, PAPI_MAX_STR_LEN - strlen(_amd_energy_vector.cmp_info.disabled_reason) - 1);
    		 _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1] = '\0';
          if (strCpy == NULL) HANDLE_STRING_ERROR;
          retval = PAPI_ESYS;
          goto fn_fail;
       }

       /* Check if a new package */
       if ((package >= 0) && (package < nr_cpus)) {
	 //se package e' ragionevole e se e' meno di nr_cpus
         if (packages[package] == -1) {
	   //se nel package letto ci sta -1, significa che ne ho trovato uno nuovo !!!!
	   printf("Found package %d out of total %d\n",package,num_packages);
	   packages[package]=package;
	   //scrivo quale cazzo di cpu utilizzare per leggere il package
	   //comunque sta cosa non ci servira' piu
	   cpu_to_use[package]=j;
	   num_packages++;
         }
       } else {
	 SUBDBG("Package outside of allowed range\n");
	 strCpy=strncpy(_amd_energy_vector.cmp_info.disabled_reason,
		"Package outside of allowed range",PAPI_MAX_STR_LEN);
	 _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
    if (strCpy == NULL) HANDLE_STRING_ERROR;
    retval = PAPI_ESYS;
    goto fn_fail;
       }

       j++;
     }
     num_cpus=j;

     if (num_packages==0) {
        SUBDBG("Can't access /dev/cpu/*/<msr_safe | msr>\n");
    strCpy=strncpy(_amd_energy_vector.cmp_info.disabled_reason,
		"Can't access /dev/cpu/*/<msr_safe | msr>",PAPI_MAX_STR_LEN);
    _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
    if (strCpy == NULL) HANDLE_STRING_ERROR;
    retval = PAPI_ESYS;
    goto fn_fail;
     }

     //printf("Found %d packages with %d cpus\n",num_packages,num_cpus);

     /* Init fd_array */
     //non ci interessa sta cosa 
     fd_array=papi_calloc(num_cpus, sizeof(struct fd_array_t));
     if (fd_array==NULL) {
       retval = PAPI_ENOMEM;
       goto fn_fail;
     }

     //Apro un FD a caso per verificare MSR, leggere le units etc...
     fd=open_fd(cpu_to_use[0]);
     if (fd<0) {
       strErr=snprintf(_amd_energy_vector.cmp_info.disabled_reason, PAPI_MAX_STR_LEN,
		       "Can't open fd for cpu0: %s",strerror(errno));
       _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN)
	 HANDLE_STRING_ERROR;
       retval = PAPI_ESYS;
       goto fn_fail;
     }

     /* Verify needed MSR is readable. In a guest VM it may not be readable*/
     if (pread(fd, &result, sizeof result, msr_rapl_power_unit) != sizeof result ) {
        strCpy=strncpy(_amd_energy_vector.cmp_info.disabled_reason,
               "Unable to access RAPL registers",PAPI_MAX_STR_LEN);
        _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
        if (strCpy == NULL) HANDLE_STRING_ERROR;
        retval = PAPI_ESYS;
        goto fn_fail;
     }

     /* Calculate the units used */
     result=read_msr(fd,msr_rapl_power_unit);

     /* units are 0.5^UNIT_VALUE */
     /* which is the same as 1/(2^UNIT_VALUE) */

     power_divisor=1<<((result>>POWER_UNIT_OFFSET)&POWER_UNIT_MASK);
     cpu_energy_divisor=1<<((result>>ENERGY_UNIT_OFFSET)&ENERGY_UNIT_MASK);

     num_events=(num_packages + num_cpus) * 2;
     
     energy_native_events = (_energy_native_event_entry_t*)
       papi_calloc(num_events, sizeof(_energy_native_event_entry_t));
     if (energy_native_events == NULL) {
       strErr=snprintf(_amd_energy_vector.cmp_info.disabled_reason, PAPI_MAX_STR_LEN,
		       "%s:%i energy_native_events papi_calloc for %lu bytes failed.", __FILE__, __LINE__, num_events*sizeof(_energy_native_event_entry_t));
       _amd_energy_vector.cmp_info.disabled_reason[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       retval = PAPI_ENOMEM;
       goto fn_fail;
     }



     
     // the first half of events is reversed for counting, the second one for values
     i = 0;
     k = num_events/2;

     /* Create Events for energy measurements */
     //se posso leggere il package
     //per ogni package

     for(j=0;j<num_packages;j++) {
       //scrivo il nome dell'evento
       strErr=snprintf(energy_native_events[i].name, PAPI_MAX_STR_LEN,
		       "PACKAGE_ENERGY_CNT:PACKAGE%d",j);
       energy_native_events[i].name[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       //scrivo la descrizione dell'evento
       strErr=snprintf(energy_native_events[i].description, PAPI_MAX_STR_LEN,
		       "Energy used in counts by chip package %d",j);
       energy_native_events[i].description[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       //questa cosa del fs offset va chiarita, penso sia ok per il package
       energy_native_events[i].fd_offset=cpu_to_use[j];
       energy_native_events[i].msr=msr_pkg_energy_status;
       energy_native_events[i].resources.selector = i + 1;
       energy_native_events[i].type=PACKAGE_ENERGY_CNT;
       energy_native_events[i].return_type=PAPI_DATATYPE_UINT64;
       //scrivo il nome dell'evento
       strErr=snprintf(energy_native_events[k].name, PAPI_MAX_STR_LEN,
		       "PACKAGE_ENERGY:PACKAGE%d",j);
       energy_native_events[i].name[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       strCpy=strncpy(energy_native_events[k].units,"nJ",PAPI_MIN_STR_LEN);
       energy_native_events[k].units[PAPI_MIN_STR_LEN-1]=0;
       if (strCpy == NULL) HANDLE_STRING_ERROR;
       //scrivo la descrizione dell'evento
       strErr=snprintf(energy_native_events[k].description, PAPI_MAX_STR_LEN,
		       "Energy used by chip package %d",j);
       energy_native_events[i].description[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       energy_native_events[k].fd_offset=cpu_to_use[j];
       energy_native_events[k].msr=msr_pkg_energy_status;
       energy_native_events[k].resources.selector = k + 1;
       energy_native_events[k].type=PACKAGE_ENERGY;
       energy_native_events[k].return_type=PAPI_DATATYPE_UINT64;

       i++;
       k++;
     }

     //ne metto uno per core invece
     for(j=0;j<num_cpus;j++) {
       //printf("Genero evento per il core %d\n",j);
       strErr=snprintf(energy_native_events[i].name, PAPI_MAX_STR_LEN,
		       "PP0_ENERGY_CNT:CORE%d",j);
       energy_native_events[i].name[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       strErr=snprintf(energy_native_events[i].description, PAPI_MAX_STR_LEN,
		       "Energy used in counts by all cores in package %d",j);
       energy_native_events[i].description[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       energy_native_events[i].fd_offset=j;
       energy_native_events[i].msr=msr_pp0_energy_status;
       energy_native_events[i].resources.selector = i + 1;
       energy_native_events[i].type=PACKAGE_ENERGY_CNT;
       energy_native_events[i].return_type=PAPI_DATATYPE_UINT64;

       strErr=snprintf(energy_native_events[k].name, PAPI_MAX_STR_LEN,
		       "PP0_ENERGY:CORE%d",j);
       energy_native_events[i].name[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       strCpy=strncpy(energy_native_events[k].units,"nJ",PAPI_MIN_STR_LEN);
       energy_native_events[k].units[PAPI_MIN_STR_LEN-1]=0;
       if (strCpy == NULL) HANDLE_STRING_ERROR;
       strErr=snprintf(energy_native_events[k].description, PAPI_MAX_STR_LEN,
		       "Energy used by all cores in package %d",j);
       energy_native_events[i].description[PAPI_MAX_STR_LEN-1]=0;
       if (strErr > PAPI_MAX_STR_LEN) HANDLE_STRING_ERROR;
       energy_native_events[k].fd_offset=j;
       energy_native_events[k].msr=msr_pp0_energy_status;
       energy_native_events[k].resources.selector = k + 1;
       energy_native_events[k].type=PACKAGE_ENERGY;
       energy_native_events[k].return_type=PAPI_DATATYPE_UINT64;

       i++;
       k++;
     }

     /* Export the total number of events available */
     _amd_energy_vector.cmp_info.num_native_events = num_events;

     _amd_energy_vector.cmp_info.num_cntrs = num_events;
     _amd_energy_vector.cmp_info.num_mpx_cntrs = num_events;


     /* Export the component id */
     _amd_energy_vector.cmp_info.CmpIdx = cidx;

  fn_exit:
    _papi_hwd[cidx]->cmp_info.disabled = retval;
     return retval;
  fn_fail:
     goto fn_exit;

}


/*
 * Control of counters (Reading/Writing/Starting/Stopping/Setup)
 * functions
 */
static int
_amd_energy_init_control_state( hwd_control_state_t *ctl)
{

  _energy_control_state_t* control = (_energy_control_state_t*) ctl;
  int i;

  for(i=0;i<AMD_ENERGY_MAX_COUNTERS;i++) {
     control->being_measured[i]=0;
  }

  return PAPI_OK;
}

static int
_amd_energy_start( hwd_context_t *ctx, hwd_control_state_t *ctl)
{
  _energy_context_t* context = (_energy_context_t*) ctx;
  _energy_control_state_t* control = (_energy_control_state_t*) ctl;
  long long now = PAPI_get_real_usec();
  int i;

  
  for( i = 0; i < AMD_ENERGY_MAX_COUNTERS; i++ ) {
     if ((control->being_measured[i]) && (control->need_difference[i])) {
        context->start_value[i]=(read_energy_value(i) & 0xFFFFFFFF);
        context->accumulated_value[i]=0;
     }
  }

  control->lastupdate = now;

  return PAPI_OK;
}

static int
_amd_energy_stop( hwd_context_t *ctx, hwd_control_state_t *ctl )
{
   /* read values */
   _energy_context_t* context = (_energy_context_t*) ctx;
   _energy_control_state_t* control = (_energy_control_state_t*) ctl;
   long long now = PAPI_get_real_usec();
   int i;
   long long temp, newstart;

   for ( i = 0; i < AMD_ENERGY_MAX_COUNTERS; i++ ) {
      if (control->being_measured[i]) {
         temp = read_energy_value(i);
         if (control->need_difference[i]) {
            temp &= 0xFFFFFFFF;
            newstart = temp;
            /* test for wrap around */
            if (temp < context->start_value[i] ) {
               SUBDBG("Wraparound!\nstart:\t%#016x\ttemp:\t%#016x",
                  (unsigned)context->start_value[i], (unsigned)temp);
               temp += (0x100000000 - context->start_value[i]);
               SUBDBG("\tresult:\t%#016x\n", (unsigned)temp);
            } else {
               temp -= context->start_value[i];
            }
            // reset the start value, add to accum, set temp for convert call.
            context->start_value[i]=newstart;
            context->accumulated_value[i] += temp;
            temp = context->accumulated_value[i];
         }
         control->count[i] = convert_energy_readings( i, temp );
      }
    }
    control->lastupdate = now;
    return PAPI_OK;
}

/* Shutdown a thread */
static int
_amd_energy_shutdown_thread( hwd_context_t *ctx )
{
  ( void ) ctx;
  return PAPI_OK;
}

int
_amd_energy_read( hwd_context_t *ctx, hwd_control_state_t *ctl,
	    long long **events, int flags)
{
    (void) flags;

    _amd_energy_stop( ctx, ctl );

    /* Pass back a pointer to our results */
    *events = ((_energy_control_state_t*) ctl)->count;

    return PAPI_OK;
}


/*
 * Clean up what was setup in  rapl_init_component().
 */
static int
_amd_energy_shutdown_component( void )
{
    int i;

    if (energy_native_events) papi_free(energy_native_events);
    if (fd_array) {
       for(i=0;i<num_cpus;i++) {
	  if (fd_array[i].open) close(fd_array[i].fd);
       }
       papi_free(fd_array);
    }

    return PAPI_OK;
}


/* This function sets various options in the component
 * The valid codes being passed in are PAPI_SET_DEFDOM,
 * PAPI_SET_DOMAIN, PAPI_SETDEFGRN, PAPI_SET_GRANUL * and PAPI_SET_INHERIT
 */

static int
_amd_energy_ctl( hwd_context_t *ctx, int code, _papi_int_option_t *option )
{
    ( void ) ctx;
    ( void ) code;
    ( void ) option;

    return PAPI_OK;
}


static int
_amd_energy_update_control_state( hwd_control_state_t *ctl,
			    NativeInfo_t *native, int count,
			    hwd_context_t *ctx )
{
  int i, index;
    ( void ) ctx;

    _energy_control_state_t* control = (_energy_control_state_t*) ctl;

    /* Ugh, what is this native[] stuff all about ?*/
    /* Mostly remap stuff in papi_internal */

    for(i=0;i<AMD_ENERGY_MAX_COUNTERS;i++) {
       control->being_measured[i]=0;
    }

    for( i = 0; i < count; i++ ) {
       index=native[i].ni_event&PAPI_NATIVE_AND_MASK;
       native[i].ni_position=energy_native_events[index].resources.selector - 1;
       control->being_measured[index]=1;

       /* Only need to subtract if it's a PACKAGE_ENERGY or ENERGY_CNT type */
       control->need_difference[index]=
	 	(energy_native_events[index].type==PACKAGE_ENERGY ||
	 	energy_native_events[index].type==PACKAGE_ENERGY_CNT);
    }

    return PAPI_OK;
}


/*
 * This function has to set the bits needed to count different domains
 * In particular: PAPI_DOM_USER, PAPI_DOM_KERNEL PAPI_DOM_OTHER
 * By default return PAPI_EINVAL if none of those are specified
 * and PAPI_OK with success
 * PAPI_DOM_USER is only user context is counted
 * PAPI_DOM_KERNEL is only the Kernel/OS context is counted
 * PAPI_DOM_OTHER  is Exception/transient mode (like user TLB misses)
 * PAPI_DOM_ALL   is all of the domains
 */

static int
_amd_energy_set_domain( hwd_control_state_t *ctl, int domain )
{
    ( void ) ctl;

    /* In theory we only support system-wide mode */
    /* How to best handle that? */
    if ( PAPI_DOM_ALL != domain )
	return PAPI_EINVAL;

    return PAPI_OK;
}


static int
_amd_energy_reset( hwd_context_t *ctx, hwd_control_state_t *ctl )
{
    ( void ) ctx;
    ( void ) ctl;

    return PAPI_OK;
}


/*
 * Native Event functions
 */
static int
_amd_energy_ntv_enum_events( unsigned int *EventCode, int modifier )
{

     int index;

     switch ( modifier ) {

	case PAPI_ENUM_FIRST:

	   if (num_events==0) {
	      return PAPI_ENOEVNT;
	   }
	   *EventCode = 0;

	   return PAPI_OK;


	case PAPI_ENUM_EVENTS:

	   index = *EventCode & PAPI_NATIVE_AND_MASK;

	   if ( index < num_events - 1 ) {
	      *EventCode = *EventCode + 1;
	      return PAPI_OK;
	   } else {
	      return PAPI_ENOEVNT;
	   }
	   break;

	default:
		return PAPI_EINVAL;
	}

	return PAPI_EINVAL;
}

/*
 *
 */
static int
_amd_energy_ntv_code_to_name( unsigned int EventCode, char *name, int len )
{

     int index = EventCode & PAPI_NATIVE_AND_MASK;

     if ( index >= 0 && index < num_events ) {
	strncpy( name, energy_native_events[index].name, len );
	return PAPI_OK;
     }

     return PAPI_ENOEVNT;
}

/*
 *
 */
static int
_amd_energy_ntv_code_to_descr( unsigned int EventCode, char *name, int len )
{
     int index = EventCode;

     if ( index >= 0 && index < num_events ) {
	strncpy( name, energy_native_events[index].description, len );
	return PAPI_OK;
     }
     return PAPI_ENOEVNT;
}

static int
_amd_energy_ntv_code_to_info(unsigned int EventCode, PAPI_event_info_t *info) 
{

  int index = EventCode;

  if ( ( index < 0) || (index >= num_events )) return PAPI_ENOEVNT;

  strncpy( info->symbol, energy_native_events[index].name, sizeof(info->symbol)-1);
  info->symbol[sizeof(info->symbol)-1] = '\0';

  strncpy( info->long_descr, energy_native_events[index].description, sizeof(info->long_descr)-1);
  info->long_descr[sizeof(info->long_descr)-1] = '\0';

  strncpy( info->units, energy_native_events[index].units, sizeof(info->units)-1);
  info->units[sizeof(info->units)-1] = '\0';

  info->data_type = energy_native_events[index].return_type;

  return PAPI_OK;
}



papi_vector_t _amd_energy_vector = {
    .cmp_info = { /* (unspecified values are initialized to 0) */
       .name = "amd_energy",
       .short_name = "amd_energy",
       .description = "Linux RAPL energy measurements",
       .version = "0.0.1",
       .default_domain = PAPI_DOM_ALL,
       .default_granularity = PAPI_GRN_SYS,
       .available_granularities = PAPI_GRN_SYS,
       .hardware_intr_sig = PAPI_INT_SIGNAL,
       .available_domains = PAPI_DOM_ALL,
    },

	/* sizes of framework-opaque component-private structures */
    .size = {
	.context = sizeof ( _energy_context_t ),
	.control_state = sizeof ( _energy_control_state_t ),
	.reg_value = sizeof ( _energy_register_t ),
	.reg_alloc = sizeof ( _energy_reg_alloc_t ),
    },
	/* function pointers in this component */
    .init_thread =          _amd_energy_init_thread,
    .init_component =       _amd_energy_init_component,
    .init_control_state =   _amd_energy_init_control_state,
    .start =                _amd_energy_start,
    .stop =                 _amd_energy_stop,
    .read =                 _amd_energy_read,
    .shutdown_thread =      _amd_energy_shutdown_thread,
    .shutdown_component =   _amd_energy_shutdown_component,
    .ctl =                  _amd_energy_ctl,

    .update_control_state = _amd_energy_update_control_state,
    .set_domain =           _amd_energy_set_domain,
    .reset =                _amd_energy_reset,

    .ntv_enum_events =      _amd_energy_ntv_enum_events,
    .ntv_code_to_name =     _amd_energy_ntv_code_to_name,
    .ntv_code_to_descr =    _amd_energy_ntv_code_to_descr,
    .ntv_code_to_info =     _amd_energy_ntv_code_to_info,
};
