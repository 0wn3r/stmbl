#include "idacim_comp.h"
#include "hal.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(idacim);

HAL_PIN(d_cmd);
HAL_PIN(q_cmd);
HAL_PIN(com_pos);
HAL_PIN(cmd_mode);
HAL_PIN(en);
HAL_PIN(en_out);

HAL_PIN(id_fb);
HAL_PIN(ud_fb);

HAL_PIN(state);
HAL_PIN(timer);

HAL_PIN(r);
HAL_PIN(l);

HAL_PIN(pp);
HAL_PIN(out_rev);

HAL_PIN(test_cur);
HAL_PIN(test_vel);

HAL_PIN(vel_fb);

HAL_PIN(pwm_volt);

HAL_PIN(cur_bw);

HAL_PIN(tmp0);
HAL_PIN(tmp1);
HAL_PIN(tmp2);
HAL_PIN(tmp3);
HAL_PIN(avg_test_volt);

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;
  PIN(test_cur)                 = 3.0;
  PIN(test_vel)                 = 50.0;
  PIN(cur_bw)                   = 1.0;
}

static void nrt(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(r)       = 0.1;
      PIN(l)       = 0.001;
      PIN(out_rev) = 0.0;
      PIN(cur_bw)  = 1.0;
      break;

    case 10:  // r, l
      PIN(state)    = 1.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      printf("Measure r, l\n");
      printf("<font color='green'>block the rotor</font>\n");
      printf("idacim0.state = 1.2 <font color='green'>to start</font>\n");
      break;

    case 14:
      printf("conf0.r = %f <font color='green'># append to config</font>\n", PIN(r));
      printf("conf0.l = %f <font color='green'># append to config</font>\n", PIN(l));
      PIN(state) = 2.0;
      break;

    case 20:  // pp
      PIN(state)    = 2.1;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;
      PIN(cmd_mode) = 0.0;

      printf("Measure polepairs\n");
      printf("<font color='green'>unblock the rotor, it will move</font>\n");
      printf("idacim0.state = 2.2 <font color='green'>to start</font>\n");
      break;

    case 24:
      printf("conf0.polecount = %f <font color='green'># append to config</font>\n", PIN(pp));
      if(PIN(out_rev) > 0.0) {
        printf("conf0.out_rev = 1 <font color='green'># append to config</font>\n");
      }
      printf("done\n");
      PIN(state) = 3.0;
      break;
  }
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct idacim_ctx_t * ctx = (struct idacim_ctx_t *)ctx_ptr;
  struct idacim_pin_ctx_t *pins = (struct idacim_pin_ctx_t *)pin_ptr;

  if(PIN(en) <= 0.0) {
    PIN(state) = 0.0;
  }

  switch((int)(PIN(state) * 10.0 + 0.5)) {
    case 0:
      PIN(en_out)   = 0.0;
      PIN(cmd_mode) = 0.0;
      PIN(timer)    = 0.0;
      PIN(d_cmd)    = 0.0;
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      if(PIN(en) > 0.0) {
        PIN(state) = 1.0;
      }
      break;

    case 12:  // r -- rotor blocked, current control: inject test_cur, measure the
              // voltage the current loop needs to sustain it (r = ud_fb / id_fb).
              // com_pos is held at 0 throughout -- an induction motor has no rotor
              // flux to align to, so the injection axis is arbitrary.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 1.0;
      PIN(q_cmd)    = 0.0;
      PIN(com_pos)  = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(r) = PIN(r) * 0.99 + PIN(ud_fb) / MAX(PIN(id_fb), 0.01) * 0.01;

      PIN(timer) += period;
      if(PIN(timer) >= 2.0) {
        PIN(timer)  = 0.0;
        PIN(state)  = 1.3;
        PIN(d_cmd)  = 0.0;
        PIN(en_out) = 0.0;
        PIN(tmp0)   = 0.0;
        PIN(tmp1)   = 0.0;

        PIN(avg_test_volt) = PIN(test_cur) * MAX(PIN(r), 0.01);
        PIN(avg_test_volt) = LIMIT(PIN(avg_test_volt), PIN(pwm_volt) / 2.0);
      }
      break;

    case 13:  // l -- rotor still blocked, voltage control: toggle the commanded
              // voltage between 1.5x and 0.5x of avg_test_volt and track the
              // resulting current transient, extracting l from dV/dI * period.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 0.0;  // volt cmd
      PIN(q_cmd)    = 0.0;
      PIN(cur_bw)   = 1.0;

      if(PIN(d_cmd) < PIN(avg_test_volt)) {
        PIN(tmp0)  = PIN(tmp0) * 0.99 + PIN(id_fb) * 0.01;
        PIN(tmp1)  = PIN(tmp1) * 0.99 + PIN(ud_fb) * 0.01;
        PIN(d_cmd) = PIN(avg_test_volt) * 1.5;
      } else {
        PIN(tmp2)  = PIN(tmp2) * 0.99 + PIN(id_fb) * 0.01;
        PIN(tmp3)  = PIN(tmp3) * 0.99 + PIN(ud_fb) * 0.01;
        PIN(d_cmd) = PIN(avg_test_volt) * 0.5;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 1.0) {
        PIN(l)      = ABS(PIN(tmp1) - PIN(tmp3)) / ABS(PIN(tmp0) - PIN(tmp2)) * period;
        PIN(timer)  = 0.0;
        PIN(state)  = 1.4;
        PIN(d_cmd)  = PIN(avg_test_volt);
        PIN(en_out) = 0.0;
      }
      break;

    case 22:  // pp -- rotor free to turn: hold q_cmd at 0 and ramp com_pos open-loop
              // at test_vel while injecting d_cmd, forcing the rotor to follow the
              // rotating stator field (same technique as a sensorless PMSM I/F
              // startup). Comparing the commanded electrical rate against the
              // measured mechanical rate gives pole pairs directly.
      PIN(en_out)   = 1.0;
      PIN(cmd_mode) = 1.0;  // cur cmd
      PIN(cur_bw)   = 100.0;
      PIN(q_cmd)    = 0.0;

      PIN(d_cmd) = PIN(test_cur);

      PIN(com_pos) += PIN(test_vel) * period;
      PIN(com_pos) = mod(PIN(com_pos));

      if(ABS(PIN(vel_fb)) > 0.1) {
        PIN(pp) = PIN(pp) * 0.995 + PIN(test_vel) / PIN(vel_fb) * 0.005;
      }

      PIN(timer) += period;
      if(PIN(timer) >= 3.0) {
        PIN(timer) = 0.0;

        if(PIN(pp) < 0.0) {
          PIN(out_rev) = 1.0;
          PIN(pp) *= -1.0;
        }
        PIN(pp) = (int)(PIN(pp) + 0.5);

        PIN(en_out)   = 0.0;
        PIN(d_cmd)    = 0.0;
        PIN(cmd_mode) = 0.0;

        PIN(state) = 2.4;
      }
      break;
  }
}

hal_comp_t idacim_comp_struct = {
    .name      = "idacim",
    .nrt       = nrt,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct idacim_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
