/*
 * rpmsg_json.c
 *
 *  Created on: Oct 4, 2014
 *      Author: etsam
 *
 * Software License Agreement (BSD License)
 *
 * ========================================
 * Copyright (c) 2014, Mentor Graphics Corporation. All rights reserved.
 * Copyright (c) 2015 - 2016 Xilinx, Inc. All rights reserved.
 * Copyright (c) 2016 Freescale Semiconductor, Inc. All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The names of its contributors may not be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * This linux userspace application is to get the benchmark
 * data from remote processors via IPC RPMsg_char channels
 * The application sends chunks of data to the
 * remote processor. The remote side replies the data back
 * with the benchmark data. The application will use the
 * obtained benchmark data to update the JSON file.
 */

#include <dirent.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <time.h>
#include <fcntl.h>
#include <string.h>
#include <linux/rpmsg.h>
#include "jsmn.h"
#include "benchmark_stat.h"

struct _payload {
  unsigned long num;
  unsigned long size;
  char data[];
};

static int charfd = -1, fd = -1, err_cnt;

struct _payload *i_payload;
struct _payload *r_payload;

#define NUM_R5_CORES 4
core_stat R5CoreStat[NUM_R5_CORES];
core_output A53CoreStat;

#define RPMSG_HEADER_LEN 16
#define MAX_RPMSG_BUFF_SIZE (512 - RPMSG_HEADER_LEN)
#define PAYLOAD_MIN_SIZE	1
#define PAYLOAD_MAX_SIZE	(MAX_RPMSG_BUFF_SIZE - 24)
#define PAYLOAD_TEST_SIZE 256
#define RPMSG_BUS_SYS "/sys/bus/rpmsg"

long diff(struct timespec start, struct timespec end)
{
  struct timespec temp;

  if ((end.tv_nsec - start.tv_nsec) < 0) {
    temp.tv_sec = end.tv_sec - start.tv_sec-1;
    temp.tv_nsec = 1000000000UL + end.tv_nsec - start.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }

  return (temp.tv_sec * 1000000UL + temp.tv_nsec / 1000);
}

static int rpmsg_create_ept(int rpfd, struct rpmsg_endpoint_info *eptinfo)
{
  int ret;

  ret = ioctl(rpfd, RPMSG_CREATE_EPT_IOCTL, eptinfo);
  if (ret)
    perror("Failed to create endpoint.\n");
  return ret;
}

static char *get_rpmsg_ept_dev_name(const char *rpmsg_char_name,
    const char *ept_name,
    char *ept_dev_name)
{
  char sys_rpmsg_ept_name_path[64];
  char svc_name[64];
  char *sys_rpmsg_path = "/sys/class/rpmsg";
  FILE *fp;
  int i;
  int ept_name_len;

  for (i = 0; i < 128; i++) {
    sprintf(sys_rpmsg_ept_name_path, "%s/%s/rpmsg%d/name",
      sys_rpmsg_path, rpmsg_char_name, i);
    printf("checking %s\n", sys_rpmsg_ept_name_path);
    if (access(sys_rpmsg_ept_name_path, F_OK) < 0)
      continue;
    fp = fopen(sys_rpmsg_ept_name_path, "r");
    if (!fp) {
      printf("failed to open %s\n", sys_rpmsg_ept_name_path);
      break;
    }
    fgets(svc_name, sizeof(svc_name), fp);
    fclose(fp);
    printf("svc_name: %s.\n",svc_name);
    ept_name_len = strlen(ept_name);
    if (ept_name_len > sizeof(svc_name))
      ept_name_len = sizeof(svc_name);
    if (!strncmp(svc_name, ept_name, ept_name_len)) {
      sprintf(ept_dev_name, "rpmsg%d", i);
      return ept_dev_name;
    }
  }

  printf("Not able to RPMsg endpoint file for %s:%s.\n",
    rpmsg_char_name, ept_name);
  return NULL;
}

static int bind_rpmsg_chrdev(const char *rpmsg_dev_name)
{
  char fpath[256];
  char *rpmsg_chdrv = "rpmsg_chrdev";
  int fd;
  int ret;

  /* rpmsg dev overrides path */
  sprintf(fpath, "%s/devices/%s/driver_override",
    RPMSG_BUS_SYS, rpmsg_dev_name);
  fd = open(fpath, O_WRONLY);
  if (fd < 0) {
    fprintf(stderr, "Failed to open %s, %s\n",
      fpath, strerror(errno));
      return -EINVAL;
  }
  ret = write(fd, rpmsg_chdrv, strlen(rpmsg_chdrv) + 1);
  if (ret < 0) {
    fprintf(stderr, "Failed to write %s to %s, %s\n",
      rpmsg_chdrv, fpath, strerror(errno));
    return -EINVAL;
  }
  close(fd);

  /* bind the rpmsg device to rpmsg char driver */
  sprintf(fpath, "%s/drivers/%s/bind", RPMSG_BUS_SYS, rpmsg_chdrv);
  fd = open(fpath, O_WRONLY);
  if (fd < 0) {
    fprintf(stderr, "Failed to open %s, %s\n",
      fpath, strerror(errno));
    return -EINVAL;
  }
  ret = write(fd, rpmsg_dev_name, strlen(rpmsg_dev_name) + 1);
  if (ret < 0) {
    fprintf(stderr, "Failed to write %s to %s, %s\n",
      rpmsg_dev_name, fpath, strerror(errno));
    return -EINVAL;
  }
  close(fd);
  return 0;
}

static int unbind_rpmsg_chrdev(const char *rpmsg_dev_name)
{
  char fpath[256];
  char *rpmsg_chdrv = "rpmsg_chrdev";
  int fd;
  int ret;

  /* bind the rpmsg device to rpmsg char driver */
  sprintf(fpath, "%s/drivers/%s/unbind", RPMSG_BUS_SYS, rpmsg_chdrv);
  fd = open(fpath, O_WRONLY);
  if (fd < 0) {
    fprintf(stderr, "Failed to open %s, %s\n", fpath, strerror(errno));
      return -EINVAL;
  }
  ret = write(fd, rpmsg_dev_name, strlen(rpmsg_dev_name) + 1);
  if (ret < 0) {
    fprintf(stderr, "Failed to write %s to %s, %s\n",
      rpmsg_dev_name, fpath, strerror(errno));
    return -EINVAL;
  }
  close(fd);
  return 0;
}

static int get_rpmsg_chrdev_fd(const char *rpmsg_dev_name,
       char *rpmsg_ctrl_name)
{
  char dpath[256];
  char fpath[280];
  char *rpmsg_ctrl_prefix = "rpmsg_ctrl";
  DIR *dir;
  struct dirent *ent;
  int fd;

  sprintf(dpath, "%s/devices/%s/rpmsg", RPMSG_BUS_SYS, rpmsg_dev_name);
  dir = opendir(dpath);
  if (dir == NULL) {
    fprintf(stderr, "Failed to open dir %s\n", dpath);
    return -EINVAL;
  }
  while ((ent = readdir(dir)) != NULL) {
    if (!strncmp(ent->d_name, rpmsg_ctrl_prefix, strlen(rpmsg_ctrl_prefix))) {
      printf("Opening file %s.\n", ent->d_name);
      sprintf(fpath, "/dev/%s", ent->d_name);
      fd = open(fpath, O_RDWR | O_NONBLOCK);
      if (fd < 0) {
        fprintf(stderr, "Failed to open rpmsg char dev %s,%s\n",
          fpath, strerror(errno));
        return fd;
      }
      sprintf(rpmsg_ctrl_name, "%s", ent->d_name);
      return fd;
    }
  }

  fprintf(stderr, "No rpmsg char dev file is found\n");
  return -EINVAL;
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
    strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}

char dataBuf[4096];
char dataBufNew[4096];
jsmn_parser p;
jsmntok_t tokenList[1024]; /* We expect no more than 1024 tokens */
FILE *fpIn, *fpOut;

int json_file_read(char *inFileName, char *buf, int size, jsmntok_t *token, int *tokenNum) 
{
  int bytes_read, r;

  memset(buf, 0, size);
  fpIn = fopen(inFileName, "r");
  bytes_read = fread(buf, 1, size, fpIn); 
  printf("Read %d bytes from %s\n", bytes_read, inFileName); 
  jsmn_init(&p);
  r = jsmn_parse(&p, buf, bytes_read, token, *tokenNum);
  if (r < 0) {
    printf("Failed to parse JSON: %d\n", r);
    return -1;
  }

  /* Assume the top-level element is an object */
  if (r < 1 || token[0].type != JSMN_OBJECT) {
    printf("Object expected\n");
    return -1;
  }

  fclose(fpIn);

  *tokenNum = r;
  return bytes_read;
}

int json_file_write(char *outFileName, char *buf, int size) 
{
  int bytes_write;

  fpOut = fopen(outFileName, "w");
  bytes_write = fwrite(buf, 1, size, fpOut);
  printf("Write %d bytes to oob_update.json\n", bytes_write); 
  fclose(fpOut);
  return bytes_write;
}

int json_read_fields(char *dataBuf, int size, jsmntok_t *t, int r, core_stat *myCoreStat, int num, core_output *myCoreOut) 
{
  int i, j, k, l, m;
  char tempBuf[128];
  char tempNum[20];

  /* Looking for each R5 core */
  for (m=0; m<4; m++)
  {
    sprintf(tempBuf, "core%d", m);
    for (i = 1; i < r; i++) 
    {
      if (jsoneq(dataBuf, &t[i], tempBuf) == 0)
      {
        /* if we got coreN */
        printf("- %s: %.*s\n", tempBuf, t[i + 1].end - t[i + 1].start,
             dataBuf + t[i + 1].start);
        i++;

        /* looking for input */
        for (j = i; j < r; j++) 
        {
          if (jsoneq(dataBuf, &t[j], "input") == 0)
          {
            /* if we got intput */
            printf("- input: %.*s\n", t[j + 1].end - t[j + 1].start,
                dataBuf + t[j + 1].start);
            j++;

            /* looking for application */
            for (k = j; k < r; k++) 
            {
              /* find application field */
              if (jsoneq(dataBuf, &t[k], "application") == 0)
              {
                /* if we got application */
                printf("- application: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                strncpy(tempNum, dataBuf + t[k + 1].start, t[k + 1].end - t[k + 1].start);
                myCoreStat[m].input.app = atoi(tempNum);
                k++;
              }

              /* find frequency field */
              if (jsoneq(dataBuf, &t[k], "frequency") == 0)
              {
                /* if we got frequency */
                printf("- frequency: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                strncpy(tempNum, dataBuf + t[k + 1].start, t[k + 1].end - t[k + 1].start);
                myCoreStat[m].input.freq = atoi(tempNum);
                k++;
              }

              /* find changed field */
              if (jsoneq(dataBuf, &t[k], "changed") == 0)
              {
                /* if we got changed */
                printf("- changed: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                strncpy(tempNum, dataBuf + t[k + 1].start, t[k + 1].end - t[k + 1].start);
                myCoreStat[m].input.mod_flag = atoi(tempNum);
                k++;
              }
            } /* k loop */  
          } /* input */
        } /* j loop */

        /* looking for output */
        for (j = i; j < r; j++) 
        {
          if (jsoneq(dataBuf, &t[j], "output") == 0)
          {
            /* if we got output */
            printf("- output: %.*s\n", t[j + 1].end - t[j + 1].start,
              dataBuf + t[j + 1].start);
            j++;

            /* looking for cpu_load */
            for (k = j; k < r; k++) 
            {
              if (jsoneq(dataBuf, &t[k], "cpu_load") == 0)
              {
                /* if we got cpu_load */
                printf("- cpu_load: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                k++;

                /* looking for current, average and max */
                for (l = k; l < r; l++) 
                {
                  /* find current field */
                  if (jsoneq(dataBuf, &t[l], "current") == 0)
                  {
                    /* if we got current */
                    printf("- current: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.cload.cur = atoi(tempNum);
                    l++;
                  }

                  /* find average field */
                  if (jsoneq(dataBuf, &t[l], "average") == 0)
                  {
                    /* if we got average */
                    printf("- average: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.cload.ave = atoi(tempNum);
                    l++;
                  }

                  /* find max field */
                  if (jsoneq(dataBuf, &t[l], "max") == 0)
                  {
                    /* if we got max */
                    printf("- max: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.cload.max = atoi(tempNum);
                    l++;
                  }
                } /* l loop */ 
              } /* cpu_load */
            } /* k loop */

            /* looking for int_latency */
            for (k = j; k < r; k++) 
            {
              if (jsoneq(dataBuf, &t[k], "int_latency") == 0)
              {
                /* if we got int_latency */
                printf("- int_latency: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                k++;

                /* looking for average and max */
                for (l = k; l < r; l++) 
                {
                  /* find average field */
                  if (jsoneq(dataBuf, &t[l], "average") == 0)
                  {
                    /* if we got average */
                    printf("- average: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.ilate.ave = atoi(tempNum);
                    l++;
                  }

                  /* find max field */
                  if (jsoneq(dataBuf, &t[l], "max") == 0) 
                  {
                    /* if we got max */
                    printf("- max: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.ilate.max = atoi(tempNum);
                    l++;
                  }
                } /* l loop */
              } /* int_latency */  
            } /* k loop */

            /* looking for cycles_per_loop */
            for (k = j; k < r; k++) 
            {
              if (jsoneq(dataBuf, &t[k], "cycles_per_loop") == 0)
              {
                /* if we got cycles_per_loop */
                printf("- cycles_per_loop: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                k++;

                /* looking for average and max */
                for (l = k; l < r; l++) 
                {
                  /* find average field */
                  if (jsoneq(dataBuf, &t[l], "average") == 0)
                  {
                    /* if we got average */
                    printf("- average: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.ccploop.ave = atoi(tempNum);
                    l++;
                  }

                  /* find max field */
                  if (jsoneq(dataBuf, &t[l], "max") == 0)
                  {
                    /* if we got max */
                    printf("- max: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                    strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                    myCoreStat[m].output.ccploop.max = atoi(tempNum);
                    l++;
                  }
                } /* l loop */
              } /* cycles_per_loop */  
            } /* k loop */

            /* looking for sram */
            for (k = j; k < r; k++) 
            {
              if (jsoneq(dataBuf, &t[k], "sram") == 0)
              {
                /* if we got sram */
                printf("- sram: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
                strncpy(tempNum, dataBuf + t[k + 1].start, t[k + 1].end - t[k + 1].start);
                myCoreStat[m].output.sram_pcnt = 100-atoi(tempNum);
                k++;
              } /* sram */
            }/* k loop */
          } /* output */ 
        } /* j loop */
      } /* coreN */
    } /* i loop */     
  } /* m loop */

  for (i = 1; i < r; i++) 
  {
    if (jsoneq(dataBuf, &t[i], "a53") == 0)
    {
      /* if we got a53 */
      printf("- %s: %.*s\n", tempBuf, t[i + 1].end - t[i + 1].start,
           dataBuf + t[i + 1].start);
      i++;

      /* looking for output */
      for (j = i; j < r; j++) 
      {
        if (jsoneq(dataBuf, &t[j], "output") == 0)
        {
          /* if we got output */
          printf("- output: %.*s\n", t[j + 1].end - t[j + 1].start,
            dataBuf + t[j + 1].start);
          j++;

          /* looking for cpu_load */
          for (k = j; k < r; k++) 
          {
            if (jsoneq(dataBuf, &t[k], "cpu_load") == 0)
            {
              /* if we got cpu_load */
              printf("- cpu_load: %.*s\n", t[k + 1].end - t[k + 1].start,
              dataBuf + t[k + 1].start);
              k++;

              /* looking for current, average and max */
              for (l = k; l < r; l++) 
              {
                /* find current field */
                if (jsoneq(dataBuf, &t[l], "current") == 0)
                {
                  /* if we got current */
                  printf("- current: %.*s\n", t[l + 1].end - t[l + 1].start,
                  dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->cload.cur = atoi(tempNum);
                  l++;
                }

                /* find average field */
                if (jsoneq(dataBuf, &t[l], "average") == 0)
                {
                  /* if we got average */
                  printf("- average: %.*s\n", t[l + 1].end - t[l + 1].start,
                  dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->cload.ave = atoi(tempNum);
                  l++;
                }

                /* find max field */
                if (jsoneq(dataBuf, &t[l], "max") == 0)
                {
                  /* if we got max */
                  printf("- max: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->cload.max = atoi(tempNum);
                  l++;
                }
              } /* l loop */ 
            } /* cpu_load */
          } /* k loop */

          /* looking for int_latency */
          for (k = j; k < r; k++) 
          {
            if (jsoneq(dataBuf, &t[k], "int_latency") == 0)
            {
              /* if we got int_latency */
              printf("- int_latency: %.*s\n", t[k + 1].end - t[k + 1].start,
              dataBuf + t[k + 1].start);
              k++;

              /* looking for average and max */
              for (l = k; l < r; l++) 
              {
                /* find average field */
                if (jsoneq(dataBuf, &t[l], "average") == 0)
                {
                  /* if we got average */
                  printf("- average: %.*s\n", t[l + 1].end - t[l + 1].start,
                  dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->ilate.ave = atoi(tempNum);
                  l++;
                }

                /* find max field */
                if (jsoneq(dataBuf, &t[l], "max") == 0)
                {
                  /* if we got max */
                  printf("- max: %.*s\n", t[l + 1].end - t[l + 1].start,
                    dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->ilate.max = atoi(tempNum);
                  l++;
                }
              } /* l loop */
            } /* int_latency */  
          } /* k loop */

          /* looking for cycles_per_loop */
          for (k = j; k < r; k++) 
          {
            if (jsoneq(dataBuf, &t[k], "cycles_per_loop") == 0)
            {
              /* if we got cycles_per_loop */
              printf("- cycles_per_loop: %.*s\n", t[k + 1].end - t[k + 1].start,
              dataBuf + t[k + 1].start);
              k++;

              /* looking for average and max */
              for (l = k; l < r; l++) 
              {
                /* find average field */
                if (jsoneq(dataBuf, &t[l], "average") == 0)
                {
                  /* if we got average */
                  printf("- average: %.*s\n", t[l + 1].end - t[l + 1].start,
                  dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->ccploop.ave = atoi(tempNum);
                  l++;
                }

                /* find max field */
                if (jsoneq(dataBuf, &t[l], "max") == 0)
                {
                  /* if we got max */
                  printf("- max: %.*s\n", t[l + 1].end - t[l + 1].start,
                  dataBuf + t[l + 1].start);
                  strncpy(tempNum, dataBuf + t[l + 1].start, t[l + 1].end - t[l + 1].start);
                  myCoreOut->ccploop.max = atoi(tempNum);
                  l++;
                }
              } /* l loop */
            } /* cycles_per_loop */  
          } /* k loop */

          /* looking for sram */
          for (k = j; k < r; k++) 
          {
            if (jsoneq(dataBuf, &t[k], "sram") == 0)
            {
              /* if we got sram */
              printf("- sram: %.*s\n", t[k + 1].end - t[k + 1].start,
                dataBuf + t[k + 1].start);
              strncpy(tempNum, dataBuf + t[k + 1].start, t[k + 1].end - t[k + 1].start);
              myCoreOut->sram_pcnt = 100-atoi(tempNum);
              k++;
            } /* sram */
          }/* k loop */
        } /* output */ 
      } /* j loop */
    } /* coreN */
  } /* i loop */     

  return EXIT_SUCCESS;
}

int json_write_fields(char *outBuf, int bufSize, core_stat *coreStat, int num, core_output *coreOut) 
{
  int i;
  int index = 0;

  /* output top level object { */
  sprintf(outBuf+index, "{\n");
  index += 2;

  /* output JSON for each core */
  for (i=0; i<num; i++)
  {
    /* output core object { */
    sprintf(outBuf+index, "  \"core%d\": {\n", i);
    index += 13;

    /* output input object { */
    sprintf(outBuf+index, "    \"input\": {\n");
    index += 15;
    /* output application field { */
    sprintf(outBuf+index, "      \"application\": %d,\n", coreStat[i].input.app);
    index += 24;
    /* output frequency field { */
    sprintf(outBuf+index, "      \"frequency\": %d,\n", coreStat[i].input.freq);
    index += 22;
    /* output changed field { */
    sprintf(outBuf+index, "      \"changed\": %d\n", coreStat[i].input.mod_flag);
    index += 19;
    /* output input object } */
    sprintf(outBuf+index, "    },\n");
    index += 7;

    /* output output object { */
    sprintf(outBuf+index, "    \"output\": {\n");
    index += 16;
    /* output cpu_load object { */
    sprintf(outBuf+index, "      \"cpu_load\": {\n");
    index += 20;
    /* output current field { */
    sprintf(outBuf+index, "        \"current\": %*d,\n", 3, coreStat[i].output.cload.cur);
    index += 24;
    /* output average field { */
    sprintf(outBuf+index, "        \"average\": %*d,\n", 3, coreStat[i].output.cload.ave);
    index += 24;
    /* output max field { */
    sprintf(outBuf+index, "        \"max\": %*d\n", 3, coreStat[i].output.cload.max);
    index += 19;
    /* output cpu_load object } */
    sprintf(outBuf+index, "      },\n");
    index += 9;

    /* output int_latency object { */
    sprintf(outBuf+index, "      \"int_latency\": {\n");
    index += 23;
    /* output average field { */
    sprintf(outBuf+index, "        \"average\": %*d,\n", 6, coreStat[i].output.ilate.ave);
    index += 27;
    /* output max field { */
    sprintf(outBuf+index, "        \"max\": %*d\n", 6, coreStat[i].output.ilate.max);
    index += 22;
    /* output int_latency object } */
    sprintf(outBuf+index, "      },\n");
    index += 9;

    /* output cycles_per_loop object { */
    sprintf(outBuf+index, "      \"cycles_per_loop\": {\n");
    index += 27;
    /* output average field { */
    sprintf(outBuf+index, "        \"average\": %*d,\n", 6, coreStat[i].output.ccploop.ave);
    index += 27;
    /* output max field { */
    sprintf(outBuf+index, "        \"max\": %*d\n", 6, coreStat[i].output.ccploop.max);
    index += 22;
    /* output cycles_per_loop object } */
    sprintf(outBuf+index, "      },\n");
    index += 9;

    /* output sram_label field { */
    sprintf(outBuf+index, "        \"sram_label\": \"OC-SRAM:   %*d%%\",\n", 3, coreStat[i].output.sram_pcnt);
    index += 41;
    /* output sram field { */
    sprintf(outBuf+index, "        \"sram\": %*d\n", 3, 100-coreStat[i].output.sram_pcnt);
    index += 20;

    /* output output object } */
    sprintf(outBuf+index, "    }\n");
    index += 6;

    /* output core object } */
    sprintf(outBuf+index, "  },\n");
    index += 5;
  }

  /* output a53 object { */
  sprintf(outBuf+index, "  \"a53\": {\n");
  index += 11;

  /* output output object { */
  sprintf(outBuf+index, "    \"output\": {\n");
  index += 16;
  /* output cpu_load object { */
  sprintf(outBuf+index, "      \"cpu_load\": {\n");
  index += 20;
  /* output current field { */
  sprintf(outBuf+index, "        \"current\": %*d,\n", 3, coreOut->cload.cur);
  index += 24;
  /* output average field { */
  sprintf(outBuf+index, "        \"average\": %*d,\n", 3, coreOut->cload.ave);
  index += 24;
  /* output max field { */
  sprintf(outBuf+index, "        \"max\": %*d\n", 3, coreOut->cload.max);
  index += 19;
  /* output cpu_load object } */
  sprintf(outBuf+index, "      },\n");
  index += 9;

  /* output int_latency object { */
  sprintf(outBuf+index, "      \"int_latency\": {\n");
  index += 23;
  /* output average field { */
  sprintf(outBuf+index, "        \"average\": %*d,\n", 6, coreOut->ilate.ave);
  index += 27;
  /* output max field { */
  sprintf(outBuf+index, "        \"max\": %*d\n", 6, coreOut->ilate.max);
  index += 22;
  /* output int_latency object } */
  sprintf(outBuf+index, "      },\n");
  index += 9;

  /* output cycles_per_loop object { */
  sprintf(outBuf+index, "      \"cycles_per_loop\": {\n");
  index += 27;
  /* output average field { */
  sprintf(outBuf+index, "        \"average\": %*d,\n", 6, coreOut->ccploop.ave);
  index += 27;
  /* output max field { */
  sprintf(outBuf+index, "        \"max\": %*d\n", 6, coreOut->ccploop.max);
  index += 22;
  /* output cycles_per_loop object } */
  sprintf(outBuf+index, "      },\n");
  index += 9;

  /* output sram_label field { */
  sprintf(outBuf+index, "        \"sram_label\": \"OC-SRAM:   %*d%%\",\n", 3, coreOut->sram_pcnt);
  index += 41;
  /* output sram field { */
  sprintf(outBuf+index, "        \"sram\": %*d\n", 3, 100-coreOut->sram_pcnt);
  index += 20;

  /* output output object } */
  sprintf(outBuf+index, "    }\n");
  index += 6;

  /* output a53 object } */
  sprintf(outBuf+index, "  }\n");
  index += 4;

  /* output top level object } */
  sprintf(outBuf+index, "}\n");
  index += 2;

  printf("Total %d bytes have output\n", index);

  return index;
}

char rpmsg_dev[256]="virtio0.ti.ipc4.ping-pong.-1.13";
int main(int argc, char *argv[])
{
  int ret, i, j, k;
  int size, bytes_rcvd, bytes_sent;
  err_cnt = 0;
  int opt;
  int ntimes = 1;
  char fpath[256];
  char jsonFilePath[256] = "usr/share/sitara-benchmark-server/app/oob_data.jason";
  char rpmsg_char_name[16];
  struct rpmsg_endpoint_info eptinfo;
  char ept_dev_name[16];
  char ept_dev_path[32];
  int token_num, bytesRead, bytesWrite;

  struct timespec start, end;
  long elapsed;
  int iter = 1, payload_test_size = PAYLOAD_MIN_SIZE;
  int *dataPtr;

  printf("\r\n RPMsg_char to JSON test start \r\n");
  
  /* do we have file path parameter */
  if ((argc==2)&&(strlen(argv[1])<256))
    strcpy(jsonFilePath, argv[1]);

  /* Load rpmsg_char driver */
  printf("\r\nMaster>probe rpmsg_char\r\n");
  ret = system("modprobe rpmsg_char");
  if (ret < 0) {
     perror("Failed to load rpmsg_char driver.\n");
     return -EINVAL;
  }

  /* read the JSON file */
  token_num = 1024;
  memset(R5CoreStat, 0, sizeof(R5CoreStat));
  memset(&A53CoreStat, 0, sizeof(A53CoreStat));
  bytesRead = json_file_read(jsonFilePath, dataBuf, 4096, tokenList, &token_num);
  /* update the core stats from JSOn file */ 
  json_read_fields(dataBuf, bytesRead, tokenList, token_num, R5CoreStat, NUM_R5_CORES, &A53CoreStat);

  for (j=0; j<NUM_R5_CORES/2; j++)
  {
    /* update the RPMgs_char device name */
    sprintf(rpmsg_dev, "virtio%d.ti.ipc4.ping-pong.-1.13", j);
    /* Binding first RPMsg_char device */
    printf("\r\n Open rpmsg dev %s! \r\n", rpmsg_dev);
    sprintf(fpath, "%s/devices/%s", RPMSG_BUS_SYS, rpmsg_dev);
    if (access(fpath, F_OK)) {
      fprintf(stderr, "Not able to access rpmsg device %s, %s\n",
        fpath, strerror(errno));
      return -EINVAL;
    }
    ret = bind_rpmsg_chrdev(rpmsg_dev);
    if (ret < 0)
      return ret;
    charfd = get_rpmsg_chrdev_fd(rpmsg_dev, rpmsg_char_name);
    if (charfd < 0)
      return charfd;

    /* Create endpoint from rpmsg char driver */
    strcpy(eptinfo.name, "rpmsg-openamp-demo-channel");
    eptinfo.src = 0;
    eptinfo.dst = 0xFFFFFFFF;
    ret = rpmsg_create_ept(charfd, &eptinfo);
    if (ret) {
      printf("failed to create RPMsg endpoint.\n");
      return -EINVAL;
    }
    if (!get_rpmsg_ept_dev_name(rpmsg_char_name, eptinfo.name, ept_dev_name))
      return -EINVAL;
    sprintf(ept_dev_path, "/dev/%s", ept_dev_name);
    fd = open(ept_dev_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
      perror("Failed to open rpmsg device.");
      close(charfd);
      return -1;
    }

    i_payload = (struct _payload *)malloc(2 * sizeof(unsigned long) + PAYLOAD_MAX_SIZE);
    r_payload = (struct _payload *)malloc(2 * sizeof(unsigned long) + PAYLOAD_MAX_SIZE);

    if (i_payload == 0 || r_payload == 0) {
      printf("ERROR: Failed to allocate memory for payload.\n");
      return -1;
    }

    clock_gettime(CLOCK_REALTIME, &start);

    for (i = 0; i < iter; i++) {
      int k;

      i_payload->num = i;
      i_payload->size = payload_test_size;

      /* Mark the data buffer. */
      memset(&(i_payload->data[0]), 0xA5, payload_test_size);

      printf("\r\n sending payload number");
      printf(" %ld of size %ld\r\n", i_payload->num,
        (2 * sizeof(unsigned long)) + PAYLOAD_TEST_SIZE);

      bytes_sent = write(fd, i_payload,
        (2 * sizeof(unsigned long)) + payload_test_size);

      if (bytes_sent <= 0) {
        printf("\r\n Error sending data");
        printf(" .. \r\n");
        break;
      }
      printf("echo test: sent : %d\n", bytes_sent);

      r_payload->num = 0;
      bytes_rcvd = read(fd, r_payload,
        (2 * sizeof(unsigned long)) + PAYLOAD_MAX_SIZE);
      while (bytes_rcvd <= 0) {
      /* usleep(10000); */
      bytes_rcvd = read(fd, r_payload,
        (2 * sizeof(unsigned long)) + PAYLOAD_MAX_SIZE);
      }
      printf(" received payload number ");
      printf("%ld of size %ld\r\n", r_payload->num, r_payload->size);

      /* print out data buffer */
      printf("\n");
      dataPtr = (int *)&r_payload->data[0];
      for (k = 0; k < r_payload->size/sizeof(int); k++) {
        printf("0x%08x\n", *dataPtr++);
      }
      printf("\n");

      /* save the RPMsg data in R5CoreStat[] */
      R5CoreStat[j].payload_num = r_payload->num;
      R5CoreStat[j].payload_size = r_payload->size;
      memcpy(&R5CoreStat[j].input, r_payload->data, r_payload->size);

      bytes_rcvd = read(fd, r_payload,
        (2 * sizeof(unsigned long)) + PAYLOAD_MAX_SIZE);
    }

    clock_gettime(CLOCK_REALTIME, &end);
    elapsed = diff(start, end);

    printf("Avg round trip time: %ld usecs\n", elapsed / iter);
    printf("\r\n **********************************");
    printf("****\r\n");

    free(i_payload);
    free(r_payload);

    close(fd);
    if (charfd >= 0)
      close(charfd);

    // Unbind chardev to be able to run this echo test again since it will
    // attempt to rebind and fail otherwise
    ret = unbind_rpmsg_chrdev(rpmsg_dev);
    if (ret < 0)
      return ret;
  }

  /* Generate JSON file using the core stats */
  bytesRead = json_write_fields(dataBufNew, 4096, R5CoreStat, NUM_R5_CORES, &A53CoreStat);
  bytesWrite = json_file_write(jsonFilePath, dataBufNew, bytesRead);    

  return 0;
}
