
#ifdef __cplusplus
extern "C" {
#endif

void cmd_notify(void);
void env_indicate(void);
void reg_notify(void);
void stderr_notify(void);
void term_notify(u8_t *data, u8_t length);
void bat_notify(void);
void quat_notify(void);
void agm_notify(void);

#ifdef __cplusplus
}
#endif
