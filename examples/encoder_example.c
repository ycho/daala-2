/* -*- Mode: C++; tab-width: 8; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*Daala video codec
Copyright (c) 2006-2010 Daala project contributors.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS”
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.*/

#ifdef HAVE_CONFIG_H
# include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <getopt.h>
#include "../src/logging.h"
#include "daala/daalaenc.h"
#if defined(_WIN32)
# include <fcntl.h>
# include <io.h>
#else
# include <unistd.h>
#endif

typedef struct av_input av_input;

struct av_input{
  int has_video;
  FILE *video_infile;
  int video_pic_w;
  int video_pic_h;
  int video_fps_n;
  int video_fps_d;
  int video_par_n;
  int video_par_d;
  int video_interlacing;
  char video_chroma_type[16];
  int video_nplanes;
  daala_plane_info video_plane_info[OD_NPLANES_MAX];
  od_img video_img;
  int video_cur_img;
};

static int y4m_parse_tags(av_input *avin, char *tags) {
  int got_w;
  int got_h;
  int got_fps;
  int got_interlacing;
  int got_par;
  int got_chroma;
  int tmp_video_fps_n;
  int tmp_video_fps_d;
  int tmp_video_par_n;
  int tmp_video_par_d;
  char *p;
  char *q;
  got_w = got_h = got_interlacing = got_chroma = 0;
  got_fps = avin->video_fps_n > 0 && avin->video_fps_d > 0;
  got_par = avin->video_par_n >= 0 && avin->video_par_d >= 0;
  for (p = tags;; p = q) {
    /*Skip any leading spaces.*/
    while (*p == ' ') p++;
    /*If that's all we have, stop.*/
    if (p[0] == '\0') break;
    /*Find the end of this tag.*/
    for (q = p + 1; *q != '\0' && *q != ' '; q++);
    /*Process the tag.*/
    switch (p[0]) {
      case 'W': {
        if (sscanf(p + 1, "%d", &avin->video_pic_w) != 1) return -1;
        got_w = 1;
        break;
      }
      break;
      case 'H': {
        if (sscanf(p + 1, "%d", &avin->video_pic_h) != 1) return -1;
        got_h = 1;
        break;
      }
      case 'F': {
        if (sscanf(p + 1, "%d:%d", &tmp_video_fps_n, &tmp_video_fps_d) != 2) {
          return -1;
        }
        got_fps = 1;
        break;
      }
      break;
      case 'I': {
        avin->video_interlacing = p[1];
        got_interlacing = 1;
        break;
      }
      break;
      case 'A': {
        if (sscanf(p + 1, "%d:%d", &tmp_video_par_n,
         &tmp_video_par_d) != 2) return -1;
        got_par = 1;
        break;
      }
      break;
      case 'C': {
        if (q - p > 16) return -1;
        memcpy(avin->video_chroma_type, p + 1, q - p - 1);
        avin->video_chroma_type[q - p - 1] = '\0';
        got_chroma = 1;
        break;
      }
      /*Ignore unknown tags.*/
    }
  }
  if (!got_w || !got_h || !got_fps || !got_interlacing || !got_par) return -1;
  /*Chroma-type is not specified in older files, e.g., those generated by
     mplayer.*/
  if (!got_chroma) strcpy(avin->video_chroma_type, "420");
  /*Update fps and aspect ration fields only if not specified on the command
     line.*/
  if (avin->video_fps_n <= 0 || avin->video_fps_d <= 0) {
    avin->video_fps_n = tmp_video_fps_n;
    avin->video_fps_d = tmp_video_fps_d;
  }
  if (avin->video_par_n < 0 || avin->video_par_d < 0) {
    avin->video_par_n = tmp_video_par_n;
    avin->video_par_d = tmp_video_par_d;
  }
  return 0;
}

static void id_y4m_file(av_input *avin, const char *file, FILE *test) {
  od_img *img;
  unsigned char buf[128];
  int ret;
  int pli;
  int bi;
  for (bi = 0; bi < 127; bi++) {
    ret = fread(buf + bi, 1, 1, test);
    if (ret < 1) return;
    if (buf[bi] == '\n') break;
  }
  if (bi >= 127) {
    fprintf(stderr, "Error parsing '%s' header; not a YUV4MPEG2 file?\n",
     file);
    exit(1);
  }
  buf[bi] = '\0';
  if (memcmp(buf, "MPEG", 4)) return;
  if (buf[4] != '2') {
    fprintf(stderr,
     "Incorrect YUV input file version; YUV4MPEG2 required.\n");
    exit(1);
  }
  ret = y4m_parse_tags(avin, (char *)buf + 5);
  if (ret < 0) {
    fprintf(stderr, "Error parsing YUV4MPEG2 header fields in '%s'.\n", file);
    exit(1);
  }
  if (avin->video_interlacing != 'p') {
    fprintf(stderr, "Interlaced input is not currently supported.\n");
    exit(1);
  }
  avin->video_infile = test;
  avin->has_video = 1;
  fprintf(stderr, "File '%s' is %ix%i %0.03f fps %s video.\n",
   file, avin->video_pic_w, avin->video_pic_h,
   (double)avin->video_fps_n/avin->video_fps_d, avin->video_chroma_type);
  /*Allocate buffers for the image data.*/
  /*TODO: Specify chroma offsets.*/
  avin->video_plane_info[0].xdec = 0;
  avin->video_plane_info[0].ydec = 0;
  if (strcmp(avin->video_chroma_type, "444") == 0) {
    avin->video_nplanes = 3;
    avin->video_plane_info[1].xdec = 0;
    avin->video_plane_info[1].ydec = 0;
    avin->video_plane_info[2].xdec = 0;
    avin->video_plane_info[2].ydec = 0;
  }
  else if (strcmp(avin->video_chroma_type, "444alpha") == 0) {
    avin->video_nplanes = 4;
    avin->video_plane_info[1].xdec = 0;
    avin->video_plane_info[1].ydec = 0;
    avin->video_plane_info[2].xdec = 0;
    avin->video_plane_info[2].ydec = 0;
    avin->video_plane_info[3].xdec = 0;
    avin->video_plane_info[3].ydec = 0;
  }
  else if (strcmp(avin->video_chroma_type, "422") == 0) {
    avin->video_nplanes = 3;
    avin->video_plane_info[1].xdec = 1;
    avin->video_plane_info[1].ydec = 0;
    avin->video_plane_info[2].xdec = 1;
    avin->video_plane_info[2].ydec = 0;
  }
  else if (strcmp(avin->video_chroma_type, "411") == 0) {
    avin->video_nplanes = 3;
    avin->video_plane_info[1].xdec = 2;
    avin->video_plane_info[1].ydec = 0;
    avin->video_plane_info[2].xdec = 2;
    avin->video_plane_info[2].ydec = 0;
  }
  else if (strcmp(avin->video_chroma_type, "420") == 0 ||
   strcmp(avin->video_chroma_type, "420jpeg") == 0 ||
   strcmp(avin->video_chroma_type, "420mpeg2") == 0 ||
   strcmp(avin->video_chroma_type, "420paldv") == 0) {
    avin->video_nplanes = 3;
    avin->video_plane_info[1].xdec = 1;
    avin->video_plane_info[1].ydec = 1;
    avin->video_plane_info[2].xdec = 1;
    avin->video_plane_info[2].ydec = 1;
  }
  else if (strcmp(avin->video_chroma_type, "mono") == 0) {
    avin->video_nplanes = 1;
  }
  else {
    fprintf(stderr, "Unknown chroma sampling type: '%s'.\n",
     avin->video_chroma_type);
    exit(1);
  }
  img = &avin->video_img;
  img->nplanes = avin->video_nplanes;
  img->width = avin->video_pic_w;
  img->height = avin->video_pic_h;
  for (pli = 0; pli < img->nplanes; pli++) {
    od_img_plane *iplane;
    iplane = img->planes + pli;
    iplane->xdec = avin->video_plane_info[pli].xdec;
    iplane->ydec = avin->video_plane_info[pli].ydec;
    iplane->xstride = 1;
    iplane->ystride = (avin->video_pic_w
     + (1 << iplane->xdec) - 1)  >>  iplane->xdec;
    iplane->data = (unsigned char *)_ogg_malloc(iplane->ystride*
     ((avin->video_pic_h + (1 << iplane->ydec) - 1)  >>  iplane->ydec));
  }
}

static void id_file(av_input *avin, const char *file) {
  unsigned char buf[4];
  FILE *test;
  int ret;
  if (!strcmp(file, "-")) test = stdin;
  else {
    test = fopen(file, "rb");
    if (test == NULL) {
      fprintf(stderr, "Unable to open input file '%s'\n", file);
      exit(1);
    }
  }
  ret = fread(buf, 1, 4, test);
  if (ret < 4) {
    fprintf(stderr, "EOF determining file type of file '%s'\n", file);
    exit(1);
  }
  if (!memcmp(buf, "YUV4", 4)) {
    if (avin->has_video) {
      fprintf(stderr,
       "Multiple YUV4MPEG2 files specified on the command line.\n");
      exit(1);
    }
    id_y4m_file(avin, file, test);
    if (!avin->has_video) {
      fprintf(stderr, "Error parsing YUV4MPEG2 file.\n");
      exit(1);
    }
  }
  else {
    fprintf(stderr,
     "Input file '%s' is not a YUV4MPEG2 file.\n", file);
  }
}

int fetch_and_process_video(av_input *avin, ogg_page *page,
 ogg_stream_state *vo, daala_enc_ctx *dd, int video_ready,
 int *limit, int *skip) {
  ogg_packet op;
  /*Last input frame to the encoder?*/
  static int last_in_frame = 0;
  /*Last output frame is emitted from encoder?*/
  static int last_frame_encoded = 0;
  while (!video_ready) {
    size_t ret;
    char frame[6];
    char c;
    if (ogg_stream_pageout(vo, page) > 0)
      return 1;
    else if (ogg_stream_eos(vo))
      return 0;
    if (!last_in_frame) {
      ret = fread(frame, 1, 6, avin->video_infile);
      if (ret == 6) {
        od_img *img;
        int pli;
        if (memcmp(frame, "FRAME", 5) != 0) {
          fprintf(stderr, "Loss of framing in YUV input data.\n");
          exit(1);
        }
        if (frame[5] != '\n') {
          int bi;
          for (bi = 0; bi < 121; bi++) {
            if (fread(&c, 1, 1, avin->video_infile) == 1 && c == '\n') break;
          }
          if (bi >= 121) {
            fprintf(stderr, "Error parsing YUV frame header.\n");
            exit(1);
          }
        }
        /*Read the frame data.*/
        img = &avin->video_img;
        for (pli = 0; pli < img->nplanes; pli++) {
          od_img_plane *iplane;
          size_t plane_sz;
          iplane = img->planes + pli;
          plane_sz = ((avin->video_pic_w + (1 << iplane->xdec) - 1)
           >> iplane->xdec)*((avin->video_pic_h + (1 << iplane->ydec)
           - 1) >> iplane->ydec);
          ret = fread(iplane->data/* + (avin->video_pic_y >> iplane->ydec)
           *iplane->ystride + (avin->video_picx >> iplane->xdec)*/, 1, plane_sz,
           avin->video_infile);
          if (ret != plane_sz) {
            fprintf(stderr, "Error reading YUV frame data.\n");
            exit(1);
          }
        }
        if (skip && (*skip) > 0) {
          (*skip)--;
          continue;
        }
        if (limit) {
          (*limit)--;
          last_in_frame = (*limit) <= 0;
        }
        else
          last_in_frame = 0;
      }
      else last_in_frame = 1;
    }
    /*Pull the packets from the previous frame, now that we know whether or not
       we can read the current one.
      This is used to set the e_o_s bit on the final packet.*/
    while (daala_encode_packet_out(dd, last_frame_encoded, &op)) {
      ogg_stream_packetin(vo, &op);
    }
    /*Submit the current frame for encoding.*/
    /*If B frames are used, then daala_encode_img_in() will buffer
       the input frames for B in in_imgs[] (i.e. frame delay),
       until it encode P frame.*/
    if (!last_frame_encoded)
      daala_encode_img_in(dd, &avin->video_img, 0, last_in_frame,
       &last_frame_encoded);
  }
  return video_ready;
}

static const char *OPTSTRING = "ho:k:v:V:s:S:l:z:";

static const struct option OPTIONS[] = {
  { "help", no_argument, NULL, 'h' },
  { "output", required_argument, NULL, 'o' },
  { "keyframe-rate", required_argument, NULL, 'k' },
  { "video-quality", required_argument, NULL, 'v' },
  { "video-rate-target", required_argument, NULL, 'V' },
  { "serial", required_argument, NULL, 's' },
  { "skip", required_argument, NULL, 'S' },
  { "limit", required_argument, NULL, 'l' },
  { "complexity", required_argument, NULL, 'z' },
  { "mc-use-chroma", no_argument, NULL, 0 },
  { "no-mc-use-chroma", no_argument, NULL, 0 },
  { "mc-use-satd", no_argument, NULL, 0 },
  { "no-mc-use-satd", no_argument, NULL, 0 },
  { "activity-masking", no_argument, NULL, 0 },
  { "no-activity-masking", no_argument, NULL, 0 },
  { "dering", no_argument, NULL, 0 },
  { "no-dering", no_argument, NULL, 0 },
  { "qm", required_argument, NULL, 0 },
  { "mv-res-min", required_argument, NULL, 0 },
  { "mv-level-min", required_argument, NULL, 0 },
  { "mv-level-max", required_argument, NULL, 0 },
  { "version", no_argument, NULL, 0},
  { NULL, 0, NULL, 0 }
};

static void usage(void) {
  fprintf(stderr,
   "Usage: encoder_example [options] video_file\n\n"
   "Options:\n\n"
   "  -h --help                      Display this help and exit.\n"
   "  -o --output <filename.ogg>     file name for encoded output;\n"
   "                                 If this option is not given, the\n"
   "                                 compressed data is written to.\n"
   "                                 a file named video_file.out.ogv.\n\n"
   "  -k --keyframe-rate <n>         Frequency of keyframes in output.\n\n"
   "  -v --video-quality <n>         Daala quality selector from 0 to 511.\n"
   "                                 511 yields the smallest files, but\n"
   "                                 lowest video quality; 1 yields the\n"
   "                                 highest quality, but large files;\n"
   "                                 0 is lossless.\n\n"
   "  -V --video-rate-target <n>     bitrate target for Daala video;\n"
   "                                 use -v and not -V if at all possible,\n"
   "                                 as -v gives higher quality for a given\n"
   "                                 bitrate. (Not yet implemented)\n\n"
   "  -s --serial <n>                Specify a serial number for the stream.\n"
   "  -S --skip <n>                  Number of input frames to skip before encoding.\n"
   "  -l --limit <n>                 Maximum number of frames to encode.\n"
   "  -z --complexity <n>            Computational complexity: 0...10\n"
   "                                 Fastest: 0, slowest: 10, default: 7\n"
   "     --[no-]mc-use-chroma        Control whether the chroma planes should\n"
   "                                 be used in the motion compensation search.\n"
   "                                 --mc-use-chroma is implied by default.\n"
   "     --[no-]mc-use-satd          Control whether the SATD metric should\n"
   "                                 be used in the motion estimation.\n"
   "                                 --no-mc-use-satd is implied by default.\n"
   "     --[no-]activity-masking     Control whether activity masking should\n"
   "                                 be used in quantization.\n"
   "     --[no-]dering               Enable (default) or disable the dering\n"
   "                                 postprocessing filter.\n"
   "     --qm <n>                    Select quantization matrix\n"
   "                                 0 => flat, 1 => hvs (default)\n"
   "                                 --activity-masking is implied by default.\n"
   "     --mv-res-min <n>            Minimum motion vectors resolution for the\n"
   "                                 motion compensation search.\n"
   "                                 0 => 1/8 pel (default), 1 => 1/4 pel,\n"
   "                                 2 => 1/2 pel\n"
   "     --mv-level-min <n>          Minimum motion vectors level between\n"
   "                                 0 (default) and 6.\n"
   "     --mv-level-max <n>          Maximum motion vectors level between\n"
   "                                 0 and 6 (default).\n"
   "     --version                   Displays version information.\n"
   " encoder_example accepts only uncompressed YUV4MPEG2 video.\n\n");
  exit(1);
}

static void version(void) {
  fprintf(stderr, "%s\n", daala_version_string());
  exit(0);
}

int main(int argc, char **argv) {
  FILE *outfile;
  av_input avin;
  ogg_stream_state vo;
  ogg_page og;
  ogg_packet op;
  daala_enc_ctx *dd;
  daala_info di;
  daala_comment dc;
  ogg_int64_t video_bytesout;
  double time_base;
  double time_spent;
  int c;
  int loi;
  int ret;
  double video_kbps;
  int video_q;
  int video_keyframe_rate;
  int video_ready;
  int pli;
  int fixedserial;
  unsigned int serial;
  int skip;
  int limit;
  int complexity;
  int interactive;
  int mc_use_chroma;
  int mc_use_satd;
  int use_activity_masking;
  int use_dering;
  int qm;
  int mv_res_min;
  int mv_level_min;
  int mv_level_max;
  int current_frame_no;
  int output_provided;
  char default_filename[1024];
  clock_t t0;
  clock_t t1;
  daala_log_init();
#if defined(_WIN32)
  _setmode(_fileno(stdin), _O_BINARY);
  _setmode(_fileno(stdout), _O_BINARY);
  interactive = _isatty(_fileno(stderr));
#else
  interactive = isatty(fileno(stderr));
#endif
  outfile = stdout;
  memset(&avin, 0, sizeof(avin));
  avin.video_fps_n = -1;
  avin.video_fps_d = -1;
  avin.video_par_n = -1;
  avin.video_par_d = -1;
  /* Set default options */
  video_q = 10;
  video_keyframe_rate = 256;
  video_bytesout = 0;
  fixedserial = 0;
  skip = 0;
  limit = -2;
  complexity = 7;
  mc_use_chroma = 1;
  mc_use_satd = 0;
  use_activity_masking = 1;
  use_dering = 1;
  qm = 1;
  mv_res_min = 0;
  mv_level_min = 0;
  mv_level_max = 6;
  output_provided = 0;
  while ((c = getopt_long(argc, argv, OPTSTRING, OPTIONS, &loi)) != EOF) {
    switch (c) {
      case 'o': {
        outfile = fopen(optarg, "wb");
        output_provided = 1;
        if (outfile == NULL) {
          fprintf(stderr, "Unable to open output file '%s'\n", optarg);
          exit(1);
        }
        break;
      }
      case 'k': {
        video_keyframe_rate = atoi(optarg);
        if (video_keyframe_rate < 1 || video_keyframe_rate > 1000) {
          fprintf(stderr,
           "Illegal video keyframe rate (use 1 through 1000)\n");
          exit(1);
        }
        break;
      }
      case 'v': {
        video_q = atoi(optarg);
        if (video_q < 0 || video_q > 511) {
          fprintf(stderr, "Illegal video quality (use 0 through 511)\n");
          exit(1);
        }
        break;
      }
      case 'V': {
        fprintf(stderr,
         "Target video bitrate is not yet implemented, use -v instead.\n");
        exit(1);
        break;
      }
      case 's': {
        if (sscanf(optarg, "%u", &serial) != 1) {
          serial = 0;
        }
        else {
          fixedserial = 1;
        }
        break;
      }
      case 'S': {
        skip = atoi(optarg);
        if (skip < 0) {
          fprintf(stderr,
           "Illegal number of frames to skip (must be non-negative)\n");
          exit(1);
        }
        break;
      }
      case 'l': {
        limit = atoi(optarg);
        if (limit < 1) {
          fprintf(stderr,
           "Illegal maximum frame limit (must be greater than 0)\n");
          exit(1);
        }
        break;
      }
      case 'z': {
        complexity = atoi(optarg);
        if (complexity < 0 || complexity > 10) {
          fprintf(stderr,
           "Illegal complexity setting (must be 0...10, inclusive)\n");
          exit(1);
        }
        break;
      }
      case 0: {
        if (strcmp(OPTIONS[loi].name, "mc-use-chroma") == 0) {
          mc_use_chroma = 1;
        }
        else if (strcmp(OPTIONS[loi].name, "no-mc-use-chroma") == 0) {
          mc_use_chroma = 0;
        }
        else if (strcmp(OPTIONS[loi].name, "mc-use-satd") == 0) {
          mc_use_satd = 1;
        }
        else if (strcmp(OPTIONS[loi].name, "no-mc-use-satd") == 0) {
          mc_use_satd = 0;
        }
        else if (strcmp(OPTIONS[loi].name, "activity-masking") == 0) {
          use_activity_masking = 1;
        }
        else if (strcmp(OPTIONS[loi].name, "no-activity-masking") == 0) {
          use_activity_masking = 0;
        }
        else if (strcmp(OPTIONS[loi].name, "dering") == 0) {
          use_dering = 1;
        }
        else if (strcmp(OPTIONS[loi].name, "no-dering") == 0) {
          use_dering = 0;
        }
        else if (strcmp(OPTIONS[loi].name, "mv-res-min") == 0) {
          mv_res_min = atoi(optarg);
          if (mv_res_min < 0 || mv_res_min > 2) {
            fprintf(stderr, "Illegal value for --mv-res-min\n");
            exit(1);
          }
        }
        else if (strcmp(OPTIONS[loi].name, "qm") == 0) {
          qm = atoi(optarg);
          if (qm < 0 || qm > 1) {
            fprintf(stderr, "Illegal value for --qm\n");
            exit(1);
          }
        }
        else if (strcmp(OPTIONS[loi].name, "mv-level-min") == 0) {
          mv_level_min = atoi(optarg);
          if (mv_level_min < 0 || mv_level_min > 6) {
            fprintf(stderr, "Illegal value for --mv-level-min\n");
            exit(1);
          }
          if (mv_level_min > mv_level_max) {
            fprintf(stderr,
             "--mv-level-min must be less than or equal to --mv-level-max\n");
            exit(1);
          }
        }
        else if (strcmp(OPTIONS[loi].name, "mv-level-max") == 0) {
          mv_level_max = atoi(optarg);
          if (mv_level_max < 0 || mv_level_max > 6) {
            fprintf(stderr, "Illegal value for --mv-level-max\n");
            exit(1);
          }
          if (mv_level_min > mv_level_max) {
            fprintf(stderr,
             "--mv-level-max must be greater than or equal to --mv-level-min\n");
            exit(1);
          }
        }
        else if (strcmp(OPTIONS[loi].name, "version") == 0) {
          version();
        }
        break;
      }
      case 'h':
      default: usage(); break;
    }
  }
  /*Assume anything following the options must be a file name.*/
  for (; optind < argc; optind++) id_file(&avin, argv[optind]);
  if(!output_provided){
    snprintf(default_filename, 1024, "%s.out.ogv", argv[argc-1]);
    outfile = fopen(default_filename, "wb");
    if (outfile == NULL) {
      fprintf(stderr, "Unable to open output file '%s'\n", default_filename);
      exit(1);
    }
  }
  if (!avin.has_video) {
    fprintf(stderr, "No video files submitted for compression.\n");
    exit(1);
  }
  if (!fixedserial) {
    srand(time(NULL));
    serial = rand();
  }
  ogg_stream_init(&vo, serial);
  daala_info_init(&di);
  di.pic_width = avin.video_pic_w;
  di.pic_height = avin.video_pic_h;
  di.timebase_numerator = avin.video_fps_n;
  di.timebase_denominator = avin.video_fps_d;
  di.frame_duration = 1;
  di.pixel_aspect_numerator = avin.video_par_n;
  di.pixel_aspect_denominator = avin.video_par_d;
  di.nplanes = avin.video_nplanes;
  memcpy(di.plane_info, avin.video_plane_info,
   di.nplanes*sizeof(*di.plane_info));
  di.keyframe_rate = video_keyframe_rate;
  /*TODO: Other crap.*/
  dd = daala_encode_create(&di);
  daala_comment_init(&dc);
  /*Set up encoder.*/
  daala_encode_ctl(dd, OD_SET_QUANT, &video_q, sizeof(video_q));
  daala_encode_ctl(dd, OD_SET_COMPLEXITY, &complexity, sizeof(complexity));
  daala_encode_ctl(dd, OD_SET_MC_USE_CHROMA, &mc_use_chroma,
   sizeof(mc_use_chroma));
  daala_encode_ctl(dd, OD_SET_MC_USE_SATD, &mc_use_satd,
   sizeof(mc_use_satd));
  daala_encode_ctl(dd, OD_SET_USE_ACTIVITY_MASKING, &use_activity_masking,
   sizeof(use_activity_masking));
  daala_encode_ctl(dd, OD_SET_USE_DERING, &use_dering,
   sizeof(use_dering));
  daala_encode_ctl(dd, OD_SET_MV_RES_MIN, &mv_res_min, sizeof(mv_res_min));
  daala_encode_ctl(dd, OD_SET_QM, &qm, sizeof(qm));
  daala_encode_ctl(dd, OD_SET_MV_LEVEL_MIN, &mv_level_min, sizeof(mv_level_min));
  daala_encode_ctl(dd, OD_SET_MV_LEVEL_MAX, &mv_level_max, sizeof(mv_level_max));
  /*Write the bitstream header packets with proper page interleave.*/
  /*The first packet for each logical stream will get its own page
     automatically.*/
  if (daala_encode_flush_header(dd, &dc, &op) <= 0) {
    fprintf(stderr, "Internal Daala library error.\n");
    exit(1);
  }
  ogg_stream_packetin(&vo, &op);
  if (ogg_stream_pageout(&vo, &og) != 1) {
    fprintf(stderr, "Internal Ogg library error.\n");
    exit(1);
  }
  if (fwrite(og.header, 1, og.header_len, outfile) < (size_t)og.header_len) {
    fprintf(stderr, "Could not complete write to file.\n");
    exit(1);
  }
  if (fwrite(og.body, 1, og.body_len, outfile) < (size_t)og.body_len) {
    fprintf(stderr, "Could not complete write to file.\n");
    exit(1);
  }
  /*Create and buffer the remaining Daala headers.*/
  for (;;) {
    ret = daala_encode_flush_header(dd, &dc, &op);
    if (ret < 0) {
      fprintf(stderr, "Internal Daala library error.\n");
      exit(1);
    }
    else if (!ret) break;
    ogg_stream_packetin(&vo, &op);
  }
  for (;;) {
    ret = ogg_stream_flush(&vo, &og);
    if (ret < 0) {
      fprintf(stderr, "Internal Ogg library error.\n");
      exit(1);
    }
    else if (!ret) break;
    if (fwrite(og.header, 1, og.header_len, outfile) < (size_t)og.header_len) {
      fprintf(stderr, "Could not write header to file.\n");
      exit(1);
    }
    if (fwrite(og.body, 1, og.body_len, outfile) < (size_t)og.body_len) {
      fprintf(stderr, "Could not write body to file.\n");
      exit(1);
    }
  }
  /*Setup complete.
     Main compression loop.*/
  fprintf(stderr, "Compressing...\n");
  video_ready = 0;
  t0 = clock();
  for (;;) {
    ogg_page video_page;
    double video_time;
    double video_fps = avin.video_fps_n/avin.video_fps_d;
    size_t bytes_written;
    video_ready = fetch_and_process_video(&avin, &video_page, &vo,
     dd, video_ready, limit > -2 ? &limit : NULL, skip > 0 ? &skip : NULL);
    /*TODO: Fetch the next video page.*/
    /*If no more pages are available, we've hit the end of the stream.*/
    if (!video_ready) break;
    video_time = daala_granule_time(dd, ogg_page_granulepos(&video_page));
    bytes_written =
     fwrite(video_page.header, 1, video_page.header_len, outfile);
    if (bytes_written < (size_t)video_page.header_len) {
      fprintf(stderr, "Could not write page header to file.\n");
      exit(1);
    }
    video_bytesout += bytes_written;
    bytes_written = fwrite(video_page.body, 1, video_page.body_len, outfile);
    if (bytes_written < (size_t)video_page.body_len) {
      fprintf(stderr, "Could not write page body to file.\n");
      exit(1);
    }
    fflush(outfile);
    video_bytesout += bytes_written;
    video_ready = 0;
    if (video_time == -1) continue;
    video_kbps = video_bytesout*8*0.001/video_time;
    time_base = video_time;
    current_frame_no = time_base*video_fps;
    if (interactive) {
      fprintf(stderr, "\r");
    }
    else {
      fprintf(stderr, "\n");
    }
    t1 = clock();
    time_spent = (double)(t1 - t0)/CLOCKS_PER_SEC;
    fprintf(stderr,
     "     %i:%02i:%02i.%02i video: %0.0fkbps - Frame %i - %0.2f FPS - %0.2f FPM     \n",
     (int)time_base/3600, ((int)time_base/60)%60, (int)time_base % 60,
     (int)(time_base*100 - (long)time_base*100), video_kbps, current_frame_no, 
     (current_frame_no)/time_spent, 
     (current_frame_no)/time_spent*60);
  }
  ogg_stream_clear(&vo);
  daala_encode_free(dd);
  daala_comment_clear(&dc);
  for (pli = 0; pli < avin.video_img.nplanes; pli++) {
    _ogg_free(avin.video_img.planes[pli].data);
  }
  if (outfile != NULL && outfile != stdout) fclose(outfile);
  fprintf(stderr, "\r    \ndone.\n\r");
  if (avin.video_infile != NULL && avin.video_infile != stdin) {
    fclose(avin.video_infile);
  }
  return 0;
}
