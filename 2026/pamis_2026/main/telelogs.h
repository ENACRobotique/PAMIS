#pragma once

#include <stddef.h>

void telelogs_init();
void telelogs_send_string(const char* str, size_t len);

void telelogs_send_float(const char* name, float value);
