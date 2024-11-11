/**
 *  @file       ExternalFlash.c
 *
 *  @brief      DataFlash Module provides an interface for DataFlash devices.
 *  @details    DataFlash Module makes use of the common bus interface, so it can be used with any DataFlash bus type (DataFlash, I2C, ...).
 *
 *  @author     Marco Di Goro
 *
 *  @copyright  Copyright 2023-17 mar 2023: 08:43:11. A-Safe Italia. All rights reserved - CONFIDENTIAL
 */
//---------------------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------------------

//-------------------------------------- Include Files ----------------------------------------------------------------

#include "ExternalFlash.h"
#include "ExternalFlash_prv.h"

#include "Callback.h"
#include "CommonInterface.h"

#include "SystemTimers.h"
#include "Utilities.h"

//-------------------------------------- PRIVATE (Variables, Constants & Defines) -------------------------------------

//! Define the callback control structure module static variable
DEFINE_CALLBACK_CONTROL_STRUCTURE(ExternalFlash_Callback_Control_Structure, EXTERNAL_FLASH_CALLBACK_REGISTERS_SIZE);

//! FLASH Instance info
#define EXTERNAL_FLASH_PAGE_SIZE_256

#define SECTOR_NUM 8


#ifdef EXTERNAL_FLASH_PAGE_SIZE_256
#define EXTERNAL_FLASH_PAGE_SIZE                256
#define EXTERNAL_FLASH_PAGE_NUMBER              1024
#define EXTERNAL_FLASH_NUMBER_OF_BYTES EXTERNAL_FLASH_PAGE_SIZE * EXTERNAL_FLASH_PAGE_NUMBER 
#define EXTERNAL_FLASH_PAGE_ADDRESS_BIT         10
#define EXTERNAL_FLASH_PAGE_ADDRESS_BYTE        2
#define EXTERNAL_FLASH_PAGE_BYTE_ADDRESS_BIT    8
#define EXTERNAL_FLASH_PAGE_BYTE_ADDRESS_BYTE   1 
#endif
#define EXTERNAL_FLASH_ADDRESS_SIZE_BYTE 3    
#define EXTERNAL_FLASH_COMMAND_SIZE_BYTE 1

#define EXTERNAL_FLASH_ADDRESS_SIZE_BYTES 3

#define EXTERNAL_FLASH_READ_DUMMY_BYES      1
#define EXTERNAL_FLASH_WRITE_DUMMY_BYTES    0
//! External Flash command opcodes

#define    EXTERNAL_FLASH_CMD_DUMMY                   	    0x00
#define    EXTERNAL_FLASH_CMD_WRITE_MEMORY    		        0x58		// WRITE Command
#define    EXTERNAL_FLASH_CMD_READ_MEMORY    		        0xd2		// READ Command
#define    EXTERNAL_FLASH_CMD_READ_STATUS_REGISTER          0xd7		// Read Status Register
#define    EXTERNAL_FLASH_CMD_INVALID                 	    0xFF

//! External Flash Message Read Header struct type
typedef __PACKED_STRUCT EXTERNAL_FLASH_READ_HEADER_STRUCT
{
    uint8_t     ExternalFlash_OpCode_Cmd         :8;
    uint8_t     ExternalFlash_Address[EXTERNAL_FLASH_ADDRESS_SIZE_BYTES + EXTERNAL_FLASH_READ_DUMMY_BYES];
} EXTERNAL_FLASH_READ_HEADER_TYPE;

//! External Flash Message Write Header struct type
typedef __PACKED_STRUCT EXTERNAL_FLASH_WRITE_HEADER_STRUCT
{
    uint8_t     ExternalFlash_OpCode_Cmd         :8;
    uint8_t     ExternalFlash_Address[EXTERNAL_FLASH_ADDRESS_SIZE_BYTES + EXTERNAL_FLASH_WRITE_DUMMY_BYTES];
} EXTERNAL_FLASH_WRITE_HEADER_TYPE;



typedef enum EXTERNAL_FLASH_STATE_ENUM
{
    EXTERNAL_FLASH_STATE_INITIALIZE,
    EXTERNAL_FLASH_STATE_IDLE,
    EXTERNAL_FLASH_STATE_SEND_READ_HEADER,
    EXTERNAL_FLASH_STATE_WAIT_SEND_READ_HEADER,
    EXTERNAL_FLASH_STATE_READ,
    EXTERNAL_FLASH_STATE_SEND_WRITE_HEADER,
    EXTERNAL_FLASH_STATE_WAIT_SEND_WRITE_HEADER,
    EXTERNAL_FLASH_STATE_WRITE,
    EXTERNAL_FLASH_STATE_SEND_READ_STATUS_REGISTER_COMMAND_BEFORE_READ,
    EXTERNAL_FLASH_STATE_READ_STATUS_REGISTER_BEFORE_READ,
    EXTERNAL_FLASH_STATE_SEND_READ_STATUS_REGISTER_COMMAND_BEFORE_WRITE,
    EXTERNAL_FLASH_STATE_READ_STATUS_REGISTER_BEFORE_WRITE,
    EXTERNAL_FLASH_STATE_INVALID
} EXTERNAL_FLASH_STATE_TYPE;

//Read command
#define EXTERNAL_FLASH_MAIN_MEMORY_PAGE_READ_COMMAND    0xd2
#define EXTERNAL_FLASH_CONTINUOUS_ARRAY_READ_HF_COMMAND 0x0b
#define EXTERNAL_FLASH_CONTINUOUS_ARRAY_READ_LF_COMMAND 0x03
#define EXTERNAL_FLASH_CONTINUOUS_ARRAY_READ_LP_COMMAND 0x01
#define EXTERNAL_FLASH_BUFFER_READ_HF_COMMAND           0xd4
#define EXTERNAL_FLASH_BUFFER_READ_LF_COMMAND           0xd1

//Write command
#define EXTERNAL_FLASH_BUFFER_WRITE_COMMAND             0x84
#define EXTERNAL_FLASH_READ_MODIFY_WRITE_COMMAND        0x58
#define EXTERNAL_FLASH_PAGE_ERASE_COMMAND               0x81
#define EXTERNAL_FLASH_BLOCK_ERASE_COMMAND              0x50
#define EXTERNAL_FLASH_SECTOR_ERASE_COMMAND             0x7c
#define EXTERNAL_FLASH_CHIP_ERASE_COMMAND               {0xc7, 0x94, 0x80, 0x9a}

//Sector Protect 
#define EXTERNAL_FLASH_SECTOR_PROTECT_COMMAND           {0x3d, 0x2a, 0x7f, 0xcf}
#define EXTERNAL_FLASH_SECTOR_PROTECT                   0xff
#define EXTERNAL_FLASH_SECTOR_UNPROTECT                 0x00
#define EXTERNAL_FLASH_SECTOR_0AUP_0BUP                 0x00
#define EXTERNAL_FLASH_SECTOR_OAP_0BP                   0x0ff
#define EXTERNAL_FLASH_SECTOR_OAP_0BUP                  0xc0
#define EXTERNAL_FLASH_SECTOR_OAUP_0BP                  0x30



static NVMEMORY_INSTANCE_TYPE ExternalFlash_Instance_Store[EXTERNAL_FLASH_CH_NUM];

//! EXTERNAL_FLASH Map struct type
typedef __PACKED_STRUCT EXTERNAL_FLASH_MAP_STRUCT
{
    EXTERNAL_FLASH_CH_TYPE      ExternalFlash_Channel;
    uint8_t                     ExternalFlash_Provider_Bound_Id;
    uint8_t                     ExternalFlash_WP_Pin;
    BOOL_TYPE                   ExternalFlash_WP_Feature;
    BOOL_TYPE                   ExternalFlash_WP_Level;
    uint8_t                     ExternalFlash_Reset_Pin;
    BOOL_TYPE                   ExternalFlash_Reset_Feature;
    BOOL_TYPE                   ExternalFlash_Reset_Level;
    GENERIC_COMM_BUS_TYPE       Generic_Comm_Bus_Id;
} EXTERNAL_FLASH_MAP_TYPE;


//External Flash Status register struct
typedef __PACKED_STRUCT EXTERNAL_FLASH_STATUS_REGISTER_STRUCT
{
    uint8_t RDY_1       : 1;
    uint8_t COMP        : 1;
    uint8_t Density     : 3;
    uint8_t Protect     : 1;
    uint8_t PageSize    : 1;
    
    uint8_t RDY_2       : 1;
    uint8_t Reserved_1  : 1;
    uint8_t EPE         : 1;
    uint8_t Reserved_2  : 5;    
}EXTERNAL_FLASH_STATUS_REGISTER_TYPE;


static EXTERNAL_FLASH_STATUS_REGISTER_TYPE Status_Register;

//! External Flash Configuration Map
static const EXTERNAL_FLASH_MAP_TYPE ExternalFlash_Map[] = EXTERNAL_FLASH_MAP; 

//! External Flash Task Handler Index
static uint8_t ExternalFlash_Handler_Index = INVALID_VALUE_8;

static uint8_t ExternalFlash_Timeout_Handle = INVALID_VALUE_8;
#define EXTERNAL_FLASH_WAIT_TIMEOUT_MS          (50)

//-------------------------------------- PRIVATE (Function Prototypes) ------------------------------------------------

static void CommBusEventHandler(CALLBACK_EVENT_TYPE event);
static void ExecuteCallBack(COMMON_I_CALLBACK_TYPE data);
static BOOL_TYPE SendCommand(uint8_t instance_id, uint8_t command_id);
static BOOL_TYPE SendReadHeader(uint8_t instance_id);
static BOOL_TYPE ReadData(uint8_t instance_id);
static BOOL_TYPE SendWriteHeader(uint8_t instace_id);
static BOOL_TYPE WriteData(uint8_t instance_id, uint16_t write_size);

//=====================================================================================================================
//-------------------------------------- Public Functions -------------------------------------------------------------
//=====================================================================================================================

void ExternalFlash__Initialize(void)
{
    // Initialize callback structure
    Callback__Initialize(&ExternalFlash_Callback_Control_Structure);

    // Start periodic handler task
    ExternalFlash_Handler_Index = SystemTimers__CreateTask("ExternalFlash__Handler", &ExternalFlash__Handler, EXTERNAL_FLASH_HANDLER_PERIOD_MS, TIMER_MS, FALSE );    // Create periodic handler
    
    SYS_ASSERT(ExternalFlash_Handler_Index != INVALID_VALUE_8);
    
    // Initialize instance id store
    memset(ExternalFlash_Instance_Store, 0x00, sizeof(ExternalFlash_Instance_Store));
    
    // Search bound bus IDs
    for(uint8_t instance_id = 0; instance_id < EXTERNAL_FLASH_CH_NUM; instance_id++)
    {
        // Get pointer to get allocation handler
        COMMBUS__GETALLOCATION alloc_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].GetAllocation;
        
        // If handler exists
        if(alloc_handler != NULL)
        {
            // Get bound instance id
            ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel = alloc_handler(ExternalFlash_Map[instance_id].ExternalFlash_Provider_Bound_Id);
        }
        
        // If External Flash instance has the write protection pin feature enabled
        if(ExternalFlash_Map[instance_id].ExternalFlash_WP_Feature == ENABLED)
        {
            // Activate Write Protection
            GENERIC_IO_HANDLERS[GENERIC_IO_DIGITALIO].Write(ExternalFlash_Map[instance_id].ExternalFlash_WP_Pin, !ExternalFlash_Map[instance_id].ExternalFlash_WP_Level);
        }
        
        // If External Flash instance has the hold pin feature enabled
        if(ExternalFlash_Map[instance_id].ExternalFlash_Reset_Feature == ENABLED)
        {
            // Activate Write Protection
            GENERIC_IO_HANDLERS[GENERIC_IO_DIGITALIO].Write(ExternalFlash_Map[instance_id].ExternalFlash_Reset_Pin, !ExternalFlash_Map[instance_id].ExternalFlash_Reset_Level);
        }
        
        // Get Register Event Handler
        COMMBUS__REGISTERHANDLER register_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].RegisterEventHandler;
        
        // If handler exists
        if(register_handler != NULL)
        {
            // Register event handler to service provider
            register_handler(CommBusEventHandler, ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, CALLBACK_FILTER_VALUE_NONE);
        }
            
        // Initialize Instance Store
        ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_INITIALIZE;
        ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_NONE;
    }
}

void ExternalFlash_Handler(void)
{
    COMMON_I_CALLBACK_TYPE nv_callback;
    
    for(uint8_t instance_id = 0; instance_id < ELEMENTS_IN_ARRAY(ExternalFlash_Instance_Store); instance_id ++)
    {
        
        COMMBUS__STOPTRANSACTION stop_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].StopTransaction;
        
        // NOTE: Only one instance can be "active", i.e. not in IDLE state, to prevent isseues when multiple channels are referred to the same physical chip
        switch(ExternalFlash_Instance_Store[instance_id].NVM_State)
        {
          case EXTERNAL_FLASH_STATE_INITIALIZE:
            
            break;
            
          case EXTERNAL_FLASH_STATE_IDLE:
            
            break;
            
          case EXTERNAL_FLASH_STATE_SEND_READ_HEADER:
            // Check if NV Process is "write complete", Header data has been transmitted
            if(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process == NVDATA_PROCESS_WRITE)
            {
                // Update NV Process Info
                ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_WAIT_READ;
                // Update Memory State machine
                ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_READ;
                        
                // Read payload data
                ReadData(instance_id);
            }
            break;
            
          case EXTERNAL_FLASH_STATE_READ:
            // Check if NV Process is "read complete", payload data has been read
            if(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process == NVDATA_PROCESS_READ)
            {
                stop_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel);   
                
                // Fill NV callback data
                nv_callback.Source_Instance_Id = instance_id;
                nv_callback.Event_Value = COMBINE_BYTES(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process,
                                                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size);
                
                // Update NV Process Info
                ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_NONE;
                // Update Memory State machine
                ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_IDLE; 
                
                // Trigger Callback Notify
                ExecuteCallBack(nv_callback);
            }
            break;
            
          case EXTERNAL_FLASH_STATE_SEND_WRITE_HEADER:
            // Check if NV Process is "write complete", Header data has been transmitted
            if(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process == NVDATA_PROCESS_WRITE)
            {
                // Calculate write size
                uint16_t write_size = MIN((ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size - ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress), EXTERNAL_FLASH_PAGE_SIZE);
                write_size = MIN((EXTERNAL_FLASH_PAGE_SIZE - ((ExternalFlash_Instance_Store[instance_id].NVM_Target_Address + ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress) % EXTERNAL_FLASH_PAGE_SIZE)), write_size);
                           
                COMMBUS__STARTTRANSACTION start_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].StartTransaction;
                if(start_handler != NULL)
                {
                    if(start_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel) == TRUE)
                    { 
                        ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_WAIT_WRITE;
                        
                        // Update Memory State machine
                        ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_WRITE;
                        // Write (partial) page
                        WriteData(instance_id, write_size);                  
                    }
                }
            }
            
            break;
            
          case EXTERNAL_FLASH_STATE_WRITE:
            // Check if NV Process is "write complete", Header data has been transmitted
            if(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process == NVDATA_PROCESS_WRITE)
            {
                stop_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel);
                if(ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress < ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size)       // If there is still something to write
                {
                    SendWriteHeader(instance_id);
                }
                else
                {
                    if(ExternalFlash_Map[instance_id].ExternalFlash_WP_Feature == ENABLED)
                    {
                        // Activate agin Write Protection
                        GENERIC_IO_HANDLERS[GENERIC_IO_DIGITALIO].Write(ExternalFlash_Map[instance_id].ExternalFlash_WP_Pin, !ExternalFlash_Map[instance_id].ExternalFlash_WP_Level);
                    }
                    
                    // Fill NV callback data
                    nv_callback.Source_Instance_Id = instance_id;
                    nv_callback.Event_Value = COMBINE_BYTES(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process,
                                                            ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size);
                    
                    // Trigger Callback Notify
                    ExecuteCallBack(nv_callback);
            
                    // Update NV Process Info
                    ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_NONE;
                    // Update Memory State machine
                    ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_IDLE;
                }
            }
            
            break;

          case EXTERNAL_FLASH_STATE_SEND_READ_STATUS_REGISTER_COMMAND:
            
            break;

          case EXTERNAL_FLASH_STATE_INVALID:
            break;
        }
    }
}


BOOL_TYPE ExternalFlash__Read(uint8_t instance_id, void* buffer, uint32_t data_address, uint16_t size)
{
    BOOL_TYPE success = FALSE;
    //If a valid instance
    if(instance_id < EXTERNAL_FLASH_CH_NUM)
    {
        //if a client has a buond bus instance
        if(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel != INVALID_VALUE_8)
        {
            //If no current process active
            if(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process == NVDATA_PROCESS_NONE &&
               ExternalFlash_Instance_Store[instance_id].NVM_State == EXTERNAL_FLASH_STATE_IDLE)
            {
                //Get pointer to start transaction handler
                COMMBUS__STARTTRANSACTION start_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].StartTransaction;
                
                //If handlers exists
                if(start_handler != NULL)
                {
                    if(start_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel) == TRUE)
                    {
                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Pointer = (uint8_t*)buffer;
                        ExternalFlash_Instance_Store[instance_id].NVM_Target_Address = ExternalFlash_Instance_Store[instance_id].NVM_Instance_Memory_Offset + data_address;
                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size = size;
                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress = 0;
                        
                        ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_WAIT_WRITE;
                        
                        ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_SEND_READ_HEADER;
                        
                        SendReadHeader(instance_id);
                    }
                }
            }
        }
    }
    return success;
}


BOOL_TYPE ExternalFlash__Write(uint8_t instance_id, void* buffer, uint32_t data_address, uint16_t size)
{
     BOOL_TYPE success = FALSE;
    
    // If a valid instnace
    if(instance_id < EXTERNAL_FLASH_CH_NUM)
    {
        // If client has a bound bus instance
        if(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel != INVALID_VALUE_8)
        {
            // If no current process active
            if(ExternalFlash_Instance_Store[instance_id].NVM_Current_Process == NVDATA_PROCESS_NONE &&
               ExternalFlash_Instance_Store[instance_id].NVM_State == EXTERNAL_FLASH_STATE_IDLE)
            {      
                // Get pointer to Start Transaction handler
                COMMBUS__STARTTRANSACTION start_handler =  GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].StartTransaction;
                
                // If handlers exist
                if(start_handler != NULL)
                {
                    // If transaction started
                    if(start_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel) == TRUE)
                    {                                     
                        // If External FLash instance has the write protection pin feature enabled
                        if(ExternalFlash_Map[instance_id].ExternalFlash_WP_Feature == ENABLED)
                        {
                            // Disable Write Protection
                            GENERIC_IO_HANDLERS[GENERIC_IO_DIGITALIO].Write(ExternalFlash_Map[instance_id].ExternalFlash_WP_Pin, ExternalFlash_Map[instance_id].ExternalFlash_WP_Level);
                        }    
                        
                        // Prepare process data
                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Pointer = (uint8_t*)buffer;
                        ExternalFlash_Instance_Store[instance_id].NVM_Target_Address = ExternalFlash_Instance_Store[instance_id].NVM_Instance_Memory_Offset + data_address;
                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size = size;
                        ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress = 0;
                        
                        // Update NV Process Info
                        ExternalFlash_Instance_Store[instance_id].NVM_Current_Process = NVDATA_PROCESS_WAIT_WRITE;

                        // Update Memory State machine
                        ExternalFlash_Instance_Store[instance_id].NVM_State = EXTERNAL_FLASH_STATE_SEND_WRITE_HEADER;
                        
                        // Send write header
                        SendWriteHeader(instance_id); 
                        
                        // Manage NV Memory RAM mirror if reference not null
                        if(ExternalFlash_Instance_Store[instance_id].NVM_Mirror_Pointer != NULL)
                        {
                            memcpy((void*)(((uint8_t*)ExternalFlash_Instance_Store[instance_id].NVM_Mirror_Pointer) + data_address), buffer, size);
                        }       
                        
                        // Resume Task to process the write request
                        SystemTimers__ResumeTask(ExternalFlash_Handler_Index);
                        
                        success = TRUE;
                    }
                }
            }
        }
    }
    
    return success;
}



uint8_t ExternalFlash__GetAllocation(uint8_t client_id, void* mirror_pointer, uint16_t nv_instance_offset)
{
    uint8_t instance_id = INVALID_VALUE_8;

    // Scroll through ExternalFlash_Map
    for(instance_id = 0; instance_id < ELEMENTS_IN_ARRAY(ExternalFlash_Map); instance_id++)
    {
        // If client id matches
        if(ExternalFlash_Map[instance_id].ExternalFlash_Channel == (EXTERNAL_FLASH_CH_TYPE)client_id)
        {
            // Chache matching instance id            
            ExternalFlash_Instance_Store[instance_id].NVM_Mirror_Pointer = mirror_pointer;    
            ExternalFlash_Instance_Store[instance_id].NVM_Instance_Memory_Offset = nv_instance_offset;                   
            break;
        }
    }
    return instance_id;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Registers event handler with the module.
 * @param   event_handler: pointer to the function that handles the event
 */
void ExternalFlash__RegisterEventHandler(CALLBACK_HANDLER_TYPE event_handler, uint16_t filter_id, uint16_t filter_value)
{
    Callback__Register(&ExternalFlash_Callback_Control_Structure, (CALLBACK_HANDLER_TYPE)event_handler, filter_id, filter_value);
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Unregisters event handler with the module.
 * @param   event_handler: pointer to the function that handles the event
 */
void ExternalFlash__UnregisterEventHandler(CALLBACK_HANDLER_TYPE event_handler)
{
    Callback__Unregister(&ExternalFlash_Callback_Control_Structure, (CALLBACK_HANDLER_TYPE)event_handler);
}

BOOL_TYPE ExternalFlash__IsBusy(uint8_t externalflash_instance)
{
    BOOL_TYPE retval = TRUE;
    return retval;
}

BOOL_TYPE ExternalFlash__CheckIntegrity(uint8_t flash_instance)
{
    BOOL_TYPE retval = TRUE;
    return retval;
}



//=====================================================================================================================
//-------------------------------------- Private Functions ------------------------------------------------------------
//=====================================================================================================================

static BOOL_TYPE SendCommand(uint8_t instance_id, uint8_t command_id)
{
    BOOL_TYPE success = TRUE;
        // Get pointer to Write handler
    COMMBUS__WRITE write_handler =  GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].Write;
        
    // If handlers exist
    if(write_handler != NULL)
    {
        // Call Handler
        if(write_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, 
                         (void*)&command_id, 
                         COMMBUS_ADDRESS_NONE, 
                         sizeof(uint8_t)) == TRUE)
        {
            if(ExternalFlash_Timeout_Handle == INVALID_VALUE_8)      // If timeout timer not allocated yet
            {
                ExternalFlash_Timeout_Handle = SystemTimers__AllocateHandle();
                SYS_ASSERT(ExternalFlash_Timeout_Handle != INVALID_VALUE_8); 
                
                // Start Timeout handle
                SystemTimers__SetMs(ExternalFlash_Timeout_Handle, EXTERNAL_FLASH_WAIT_TIMEOUT_MS);
            }
    
            // Signal ExternalFlash request success
            success = TRUE;
        }
    }
    return success;
}

//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      This function sends the Read Header to External FLash memory using the selected bus
 *
 *  @param      instance_id : specific External FLash instance
 *  @return     TRUE if Read process was successful, FALSE otherwise
 */
static BOOL_TYPE SendReadHeader(uint8_t instance_id)
{
    static EXTERNAL_FLASH_READ_HEADER_TYPE header;
    BOOL_TYPE success = FALSE;
    uint16_t page_address;
    uint8_t  byte_address;
    
    page_address = ExternalFlash_Instance_Store[instance_id].NVM_Target_Address / EXTERNAL_FLASH_PAGE_NUMBER;
    byte_address = ExternalFlash_Instance_Store[instance_id].NVM_Target_Address % EXTERNAL_FLASH_PAGE_NUMBER;
    header.ExternalFlash_OpCode_Cmd = EXTERNAL_FLASH_CMD_READ_MEMORY;
    
    memcpy(header.ExternalFlash_Address + EXTERNAL_FLASH_PAGE_ADDRESS_BYTE, (void*)&byte_address, EXTERNAL_FLASH_PAGE_BYTE_ADDRESS_BYTE);
    memcpy(header.ExternalFlash_Address, (void*)&page_address, EXTERNAL_FLASH_PAGE_ADDRESS_BYTE);
    
    // Get pointer to Write handler
    COMMBUS__WRITE write_handler =  GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].Write;
        
    // If handlers exist
    if(write_handler != NULL)
    {
        // Call Handler
        if(write_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, 
                         (void*)&header, 
                         COMMBUS_ADDRESS_NONE, 
                         sizeof(EXTERNAL_FLASH_READ_HEADER_TYPE)) == TRUE)
        {
            if(ExternalFlash_Timeout_Handle == INVALID_VALUE_8)      // If timeout timer not allocated yet
            {
                ExternalFlash_Timeout_Handle = SystemTimers__AllocateHandle();
                SYS_ASSERT(ExternalFlash_Timeout_Handle != INVALID_VALUE_8); 
                
                // Start Timeout handle
                SystemTimers__SetMs(ExternalFlash_Timeout_Handle, EXTERNAL_FLASH_WAIT_TIMEOUT_MS);
            }
    
            // Signal ExternalFlash request success
            success = TRUE;
        }
    }
    
    return success;
}

static BOOL_TYPE ReadData(uint8_t instance_id)
{
    BOOL_TYPE success = FALSE;
    
    // Get pointer to Read handler
    COMMBUS__READ read_handler =  GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].Read;
        
    // If handlers exist
    if(read_handler != NULL)
    {
        // Call Handler
        if(read_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, 
                         (void*)ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Pointer, 
                         COMMBUS_ADDRESS_NONE, 
                         ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Size) == TRUE)
        {
            if(ExternalFlash_Timeout_Handle == INVALID_VALUE_8)      // If timeout timer not allocated yet
            {
                ExternalFlash_Timeout_Handle = SystemTimers__AllocateHandle();
                SYS_ASSERT(ExternalFlash_Timeout_Handle != INVALID_VALUE_8); 
                
                // Start Timeout handle
                SystemTimers__SetMs(ExternalFlash_Timeout_Handle, EXTERNAL_FLASH_WAIT_TIMEOUT_MS);
            }
            
            // Signal External Flash request success
            success = TRUE;
        }
    }   
    return success;
}
//---------------------------------------------------------------------------------------------------------------------
/**
 *  @brief      This function sends the Write Header to External FLash memory using the selected bus
 *
 *  @param      instance_id : specific External FLash instance
 *  @return     TRUE if Write process was successful, FALSE otherwise
 */
static BOOL_TYPE SendWriteHeader(uint8_t instance_id)
{
    static EXTERNAL_FLASH_WRITE_HEADER_TYPE header;
    BOOL_TYPE success = FALSE;
    uint16_t page_address;
    uint8_t  byte_address;
    
    header.ExternalFlash_OpCode_Cmd = EXTERNAL_FLASH_CMD_WRITE_MEMORY;
    
    page_address = (ExternalFlash_Instance_Store[instance_id].NVM_Target_Address + ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress) / EXTERNAL_FLASH_PAGE_NUMBER;
    byte_address = (ExternalFlash_Instance_Store[instance_id].NVM_Target_Address + ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress) % EXTERNAL_FLASH_PAGE_NUMBER;
    header.ExternalFlash_OpCode_Cmd = EXTERNAL_FLASH_CMD_READ_MEMORY;
    
    memcpy(header.ExternalFlash_Address + EXTERNAL_FLASH_PAGE_ADDRESS_BYTE, (void*)&byte_address, EXTERNAL_FLASH_PAGE_BYTE_ADDRESS_BYTE);
    memcpy(header.ExternalFlash_Address, (void*)&page_address, EXTERNAL_FLASH_PAGE_ADDRESS_BYTE);
    
    // Get pointer to Write handler
    COMMBUS__WRITE write_handler =  GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].Write;
    
    // If handlers exist
    if(write_handler != NULL)
    {
        // Call Handler
        if(write_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, 
                         (void*)&header, 
                         COMMBUS_ADDRESS_NONE, 
                         sizeof(EXTERNAL_FLASH_WRITE_HEADER_TYPE)) == TRUE)
        {
            if(ExternalFlash_Timeout_Handle == INVALID_VALUE_8)      // If timeout timer not allocated yet
            {
                ExternalFlash_Timeout_Handle = SystemTimers__AllocateHandle();
                SYS_ASSERT(ExternalFlash_Timeout_Handle != INVALID_VALUE_8); 
                
                // Start Timeout handle
                SystemTimers__SetMs(ExternalFlash_Timeout_Handle, EXTERNAL_FLASH_WAIT_TIMEOUT_MS);
            }
    
            // Signal External FLash request success
            success = TRUE;
        }
    }
    
    return success;
}

static BOOL_TYPE WriteData(uint8_t instance_id, uint16_t write_size)
{
    BOOL_TYPE success = FALSE;
 
    // Get pointer to Write handler
    COMMBUS__WRITE write_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].Write;
    

    if(write_handler != NULL)
    {
        // Call Handler
        if(write_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, 
                         (void*)(ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Pointer + ExternalFlash_Instance_Store[instance_id].NVM_Buffer_Progress), 
                         COMMBUS_ADDRESS_NONE, 
                         write_size) == TRUE)
        {
            if(ExternalFlash_Timeout_Handle == INVALID_VALUE_8)      // If timeout timer not allocated yet
            {
                ExternalFlash_Timeout_Handle = SystemTimers__AllocateHandle();
                SYS_ASSERT(ExternalFlash_Timeout_Handle != INVALID_VALUE_8); 
                
                // Start Timeout handle
                SystemTimers__SetMs(ExternalFlash_Timeout_Handle, EXTERNAL_FLASH_WAIT_TIMEOUT_MS);
            }
        
            // Signal External Flash request success
            success = TRUE;
        }
    }
    
    return success;
}

static BOOL_TYPE ReadStatusRegister(uint8_t instance_id)
{
    BOOL_TYPE success = FALSE;
    
    COMMBUS__WRITE read_handler = GENERIC_COMM_BUS_HANDLERS[ExternalFlash_Map[instance_id].Generic_Comm_Bus_Id].Read;
    if(read_handler != NULL)
    {
        if(read_handler(ExternalFlash_Instance_Store[instance_id].Bus_Instance_Channel, 
                         (void*)&Status_Register, 
                         COMMBUS_ADDRESS_NONE, 
                         sizeof(EXTERNAL_FLASH_STATUS_REGISTER_TYPE) == TRUE))
        {
                    
            if(ExternalFlash_Timeout_Handle == INVALID_VALUE_8)      // If timeout timer not allocated yet
            {
                ExternalFlash_Timeout_Handle = SystemTimers__AllocateHandle();
                SYS_ASSERT(ExternalFlash_Timeout_Handle != INVALID_VALUE_8); 
                
                // Start Timeout handle
                SystemTimers__SetMs(ExternalFlash_Timeout_Handle, EXTERNAL_FLASH_WAIT_TIMEOUT_MS);
            }
            
            // Signal External Flash request success
            success = TRUE;
        }
    }
    return success;
}

void CommBusEventHandler(CALLBACK_EVENT_TYPE event)
{

    COMMON_I_CALLBACK_TYPE bus_event;
    BOOL_TYPE event_match = FALSE;
    
    memcpy(&bus_event, &event, sizeof(CALLBACK_EVENT_TYPE));
    
    COMM_BUS_PROCESS_TYPE bus_process = HIBYTE(bus_event.Event_Value);
    uint8_t bus_length = LOBYTE(bus_event.Event_Value);

    // Find instance linked to comm bus instance that is notifying the eveny
    for(uint8_t channel_index = 0; channel_index < EXTERNAL_FLASH_CH_NUM; channel_index++)
    {
        // If Generic Bus provider that fired the callback is the same linked to the indexed element of the map
        if(bus_event.Generic_Provider_Id == ExternalFlash_Map[channel_index].Generic_Comm_Bus_Id)
        {
            if(ExternalFlash_Instance_Store[channel_index].Bus_Instance_Channel == bus_event.Source_Instance_Id)
            {                
                switch(ExternalFlash_Instance_Store[channel_index].NVM_Current_Process)
                {
                  case NVDATA_PROCESS_WAIT_READ:
                    // Set pending read to be handled in Periodic Handler
                    ExternalFlash_Instance_Store[channel_index].NVM_Current_Process = NVDATA_PROCESS_READ;
                    SystemTimers__SetTaskIdxNextCall(ExternalFlash_Handler_Index, TASK_IMMEDIATE_EXECUTION);    // Request immediate execution on next turn
                    break;
                    
                  case NVDATA_PROCESS_WAIT_WRITE:
                    // Set pending write to be handled in Periodic Handler
                    ExternalFlash_Instance_Store[channel_index].NVM_Current_Process = NVDATA_PROCESS_WRITE;
                    SystemTimers__SetTaskIdxNextCall(ExternalFlash_Handler_Index, TASK_IMMEDIATE_EXECUTION);    // Request immediate execution on next turn
                    break;
                    
                  default:
                    // Do nothing with this instance, continue to the next one
                    continue;
                    break;
                }
                
                event_match = TRUE;
            }
            
            // Break the loop in case of event match found
            if(event_match == TRUE)
            {
                ExternalFlash_Instance_Store[channel_index].NVM_Buffer_Progress += (bus_length);
                // Reset and Release Timeout Timer
                SystemTimers__ReleaseHandle(ExternalFlash_Timeout_Handle);
                ExternalFlash_Timeout_Handle = INVALID_VALUE_8;
                break;
            }
        }
    }
}

//---------------------------------------------------------------------------------------------------------------------
/**
 * @brief   Executes a callback when External Flash generates an event
 *
 * @param   NVDATA_CALLBACK_TYPE data - Callback data
 */
static void ExecuteCallBack(COMMON_I_CALLBACK_TYPE data)
{
    CALLBACK_EVENT_TYPE callback_event;
    
    data.Generic_Provider_Id = GENERIC_NVDATA_EXTERNAL_FLASH;
    
    memcpy(&callback_event, ((uint32_t*)(&data)), sizeof(CALLBACK_EVENT_TYPE));   // Fill the common callback data type

    Callback__Notify(&ExternalFlash_Callback_Control_Structure, (CALLBACK_EVENT_TYPE)callback_event, data.Source_Instance_Id, NULL);
}

