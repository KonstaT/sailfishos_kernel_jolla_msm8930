
#ifndef __LINUX_CAPELLA_CM3602_H                                            
#define __LINUX_CAPELLA_CM3602_H      



#ifdef __KERNEL__                                                           
                           
int led_wled_set_backlight(enum led_brightness value);
int switch_wled(int led_num , int enable);
                                                              
#endif /* __KERNEL__ */                                                     
                                                                            
#endif      