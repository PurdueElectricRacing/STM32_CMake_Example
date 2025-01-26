#ifndef __DAQ_SD_H__
#define __DAQ_SD_H__

void sd_update_connection_state(void);
void sd_create_file_periodic(void);
void sd_write_periodic(void);

bool get_log_enable(void);
bool daq_request_sd_mount(void);

#endif // __DAQ_SD_H__
