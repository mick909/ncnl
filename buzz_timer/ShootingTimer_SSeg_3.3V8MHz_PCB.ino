// 7SegDuino利用シューティングタイマースケッチ
//   外付けブザー仕様（PD5:Highでオフ/Lowでオン）
//
//   ボード：専用PCB / ATMega328 (or 168) / 8MHz / 3.3V
//           外付けクリスタル, 8MHz, 16kCK, 65mS
//
// 入力：
//  ADC1 : センサー入力 -> ADMUXを通してComparatorのAIN1へ
//   PC2 : スタートボタン   (内部プルアップ)
//   PC3 : 表示ボタン      (内部プルアップ)
//   PC4 : モードSW        (内部プルアップ)
//
// 出力：
//  JP3 1(PD5 ) : ブザートリガ出力 (High:オフ / Low:オン)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

// 動作状態
#define IDLE    0
#define DELAY   1
#define RUNNING 2
#define SLEEP   5

volatile uint8_t state = SLEEP, next = IDLE;

volatile uint8_t hitDisp = 0;    // 表示しているタイムのインデックス
volatile uint8_t hitNum  = 0;    // 計測したタイムの総数

// 計測しているタイムの各桁の数値(BCDで4桁)
volatile uint32_t countTime = 0;

volatile uint8_t preStart = 0;

// 計測したタイムを格納する配列
// index = 0は0表示固定
// index = 1〜9に、9個までのタイムを格納する
volatile uint32_t hitTimes[10] = {0};

// 7セグメント表示用 ==============================================

// 7セグメントLEDに表示するデータ（フォントデータ）
// PORTB, PORTC, PORTDの順
volatile uint8_t segData[3*4] = {0};

static const uint8_t segNumFont[3*10] = {
//   --D.CG-B     -------E     ---A-F--
  ~0b00101001, ~0b00000001, ~0b00010100,  // 0
  ~0b00001001, ~0b00000000, ~0b00000000,  // 1
  ~0b00100101, ~0b00000001, ~0b00010000,  // 2
  ~0b00101101, ~0b00000000, ~0b00010000,  // 3
  ~0b00001101, ~0b00000000, ~0b00000100,  // 4
  ~0b00101100, ~0b00000000, ~0b00010100,  // 5
  ~0b00101100, ~0b00000001, ~0b00010100,  // 6
  ~0b00001001, ~0b00000000, ~0b00010100,  // 7
  ~0b00101101, ~0b00000001, ~0b00010100,  // 8
  ~0b00101101, ~0b00000000, ~0b00010100   // 9
};

// BCDを7セグの表示用フォントバッファに設定する
void setTime(int32_t time);

// ボタン処理 =====================================
volatile uint8_t startSw = 0;
volatile uint8_t countSw = 0;

#define isPushed(sw) ((sw == 0x04) ? (sw=0, 1) : 0)
#define resetSw(sw)  {sw = 0;}

// 10msイベント処理 ================================
#define COUNT_TIMER 0
#define BEEP_TIMER  1
#define BLINK_TIMER 2
#define DELAY_TIMER 3
#define POWERDOWN_TIMER 4
#define IGNORE_TIMER 5
#define TIMER_NUM_MAX 6

struct timer_t {
  uint16_t interval;        // タイマー間隔(10ms単位)
  void (*func)(void);       // タイマー完了時に呼ぶ関数
  uint16_t count;           // 内部カウント用
  uint8_t  repeat;          // 繰り返し動作フラグ(非0で繰り返し)
};

void countFunc(void);
void stopBeep(void);
void blinkFunc(void);
void delayFunc(void);
void powerDownFunc(void);
void enableComparator(void);

// タイマー情報の格納用　このインデックス値がタイマーIDとなる
volatile timer_t timer_table[TIMER_NUM_MAX] = {
  {    1, countFunc,        0, 1},
  {   30,  stopBeep,        0, 0},
  {   50, blinkFunc,        0, 1},
  {    0, delayFunc,        0, 0},
  { 3000, powerDownFunc,    0, 0},
  {    5, enableComparator, 0, 0}
};

// 現在稼働中のタイマーを格納する　格納値はタイマーID(timer_tableのインデックス)
volatile uint8_t current_timer_num = 0;
volatile uint8_t current_timer[TIMER_NUM_MAX];

// タイマーIDを指定してタイマーをリセットスタートする
inline void startTimer(uint8_t id) {
  // ここが呼ばれるときは割禁中

  timer_table[id].count = 0;

  for (uint8_t i=0; i<current_timer_num; ++i) {
    if (current_timer[i] == id) {
      return;
    }
  }
  current_timer[current_timer_num++] = id;
}

// タイマーIDとカウント値を指定してタイマーをリセットスタートする
inline void startTimer(uint8_t id, uint16_t ival) {
  // ここが呼ばれるときは割禁中

  timer_table[id].count = 0;
  timer_table[id].interval = ival;

  for (uint8_t i=0; i<current_timer_num; ++i) {
    if (current_timer[i] == id) {
      return;
    }
  }
  current_timer[current_timer_num++] = id;
}

// Timer1割り込み
ISR(TIMER1_COMPA_vect) {
  // 稼働中のタイマー情報を操作してカウントし、指定値になったら指定関数を呼ぶ
  for (uint8_t i=0; i<current_timer_num; ++i) {
    volatile timer_t* tp = &(timer_table[current_timer[i]]);

    if (++(tp->count) >= tp->interval) {
      tp->count = 0;
      (*(tp->func))();

      // 一回で終わるタイマーを取り除く
      if (tp->repeat == 0) {
        for (uint8_t idx=i--; idx<current_timer_num-1; ++idx) {
          current_timer[idx] = current_timer[idx+1];
        }
        --current_timer_num;
      }

    }

  }

  // スイッチ情報の更新
  startSw = ((startSw << 1) & 0x06) | ((PINC >> 2) & 0x01);
  countSw = ((countSw << 1) & 0x06) | ((PINC >> 3) & 0x01);
}

// ビープ音 =======================================

inline void startBeep(void) {
  // ここが呼ばれるときは割禁中

  PORTD &= ~0b00100000;

  // 0.3秒後にブザーを止めるためのタイマー
  startTimer(BEEP_TIMER);
}

// BEEP_TIMER完了割り込み処理
void stopBeep(void) {
  PORTD |= 0b00100000;
}

// 初期化 =========================================
void setup() {
  // Power Off : TWI, SPI, USART, ADC, TIMER0
  // センサーを有効にする際はADCをオン(bit0 = 0)にすること
  PRR = 0b10100111;

  // PB7 : Xtal
  // PB6 : Xtal
  // PB5 : (O) 7Seg 'D'
  // PB4 : (O) 7Seg 'DP'
  // PB3 : (O) 7Seg 'C'
  // PB2 : (O) 7Seg 'G'
  // PB1 : (O) 7Seg DIG-4
  // PB0 : (O) 7Seg 'B'
  PORTB = 0b00111101;
  DDRB  = 0b00111111;

  // PC6 : Reset
  // PC5 : (I) NC
  // PC4 : (I) モードボタン
  // PC3 : (I) 表示ボタン     -> PCINT11へ
  // PC2 : (I) スタートボタン  -> PCINT10へ
  // PC1 : (I) ADC1           -> AIN1へ
  // PC0 : (O) 7Seg 'E'
  PORTC = 0b01111101;
  DDRC  = 0b00000001;

  // PD7 : (O) 7Seg DIG-3
  // PD6 : (O) 7Seg DIG-2
  // PD5 : (O) Buzzer Trigger
  // PD4 : (O) 7Seg 'A'
  // PD3 : (O) 7Seg DIG-1
  // PD2 : (O) 7Seg 'F'
  // PD1 : (I) (NC, TxD)
  // PD0 : (I) (NC, RxD)
  PORTD = 0b00110111;
  DDRD  = 0b11111100;

  // Timer1設定
  // 10msの基本動作タイミング
  // CTC動作
  // 8MHz / 64 / 1250 = 100Hz (10ms)
  TCCR1A = 0;        // 0クリアしてから設定しないとうまく動作しない
  TCCR1B = 0;        //（Arduino環境)
  OCR1A = 1249;
  TCCR1B = 0b00001011;
  TIMSK1 = 0b00000010;

  // Timer2設定
  // LEDのダイナミック点灯(4ms)
  // CTC動作
  // 8MHz / 1024 / 31 = 252Hz (4ms)
  TCCR2A = 0;        // 0クリアしてから設定しないとうまく動作しない
  TCCR2B = 0;        //（Arduino環境)
  OCR2A = 30;
  TCCR2A = 0b00000010;
  TCCR2B = 0b00001111;
  TIMSK2 = 0b00000010;

  setTime(0);

  next = IDLE;
}

// 7セグメント関連処理 ==============================

// 7セグメント表示用配列に、数値のフォントを設定する
inline void setTime(int32_t time) {
  volatile uint8_t* dp = segData;

  volatile const uint8_t* fp = segNumFont + ((time & 0x000f) * 3);
  *dp++ = *fp++;
  *dp++ = *fp++;
  *dp++ = *fp;
  time >>= 4;

  fp = segNumFont + ((time & 0x000f) * 3);
  *dp++ = *fp++;
  *dp++ = *fp++;
  *dp++ = *fp;
  time >>= 4;

  fp = segNumFont + ((time & 0x000f) * 3);
  *dp++ = *fp++;
  *dp++ = *fp++;
  *dp++ = *fp;
  time >>= 4;

  if (time != 0) {
    fp = segNumFont + ((time & 0x000f) * 3);
    *dp++ = *fp++;
    *dp++ = *fp++;
    *dp   = *fp;
  } else {
    *dp++ = 0xff;
    *dp++ = 0xff;
    *dp   = 0xff;
  }
  segData[6] &= ~0b00010000;
}

// 点滅処理のタイマ関数
// 指定時間間隔で、点滅フラグを設定する
// (実際の点滅処理は7セグ表示処理側)
volatile uint8_t dot = 1;

void blinkFunc(void) {
  dot ^= 1;
}

// 7セグメントのダイナミック表示をドライブ
volatile uint8_t row = 0;
volatile uint8_t *sdrp = segData;

// Timer2割り込み
ISR(TIMER2_COMPA_vect) {
  // DIG-1〜4をオフ
  PORTB &= ~0b00000010;
  PORTD &= ~0b11001000;

  // 7セグフォントをピンに出力する
  // （アノードコモンなので、フォント側は不論理）
  PORTB |= 0b00111101;
  PORTC |= 0b00000001;
  PORTD |= 0b00010100;

  PORTB &= *sdrp++;
  PORTC &= *sdrp++;
  PORTD &= *sdrp++;

  // 該当の桁のアノードにHigh出力
  // ついでに2ndBeep設定中の桁の点滅と、2桁目のピリオドの点滅を処理
  switch (row++) {
    case 0:
      PORTB |= 0b00000010;
      break;
    case 1:
      PORTD |= 0b10000000;
      break;
    case 2:
      PORTD |= 0b01000000;
      if (!dot) {
        PORTB |= 0b00010000;
      }
      break;
    case 3:
      PORTD |= 0b00001000;

      // 桁をオーバーラップする（注意！）
      row = 0;
      sdrp = segData;
      break;
  }

}

// アナログコンパレータ（センサー）関連 ================
inline void enableComparator(void) {
  // ADC電源オン
  PRR  &= ~0b00000001;

  // アナログコンパレータDisable
  ACSR   = 0b10000000;

  // ADC1をAIN1に接続
  ADCSRA = 0;             // ADC Disable (for AC MUX設定)
  ADCSRB = 0b01000000;    // ACME = 1
  ADMUX &= 0b11110000;
  ADMUX |= 0b00000001;

  ACSR &= 0b11110111;   // ACIE Clear (ACDを操作するため)
  ACSR  = 0b01010010;   // AC Enable, AIN0 <- BandGap, ACI clear, ACIS10 = 下降検知
  ACSR |= 0b00001000;   // ACIE enable
}

inline void disableComparator(void) {
  ACSR &= 0b11110111;   // ACIE Clear (ACDを操作するため)
  ACSR  = 0b11010010;   // AC Disable, ACI clear

  // ADC電源オフ
  PRR  |= 0b00000001;
}

ISR(ANALOG_COMP_vect) {
  disableComparator();

  // 9個計測するまではタイムを格納する
  if (hitNum < 9) {
    if (hitNum == 0) {
      hitDisp = 1;
      setTime(countTime);
    }
    hitTimes[++hitNum] = countTime;
  }

  // 9個計測していないなら、センサー再起動タイマを設定する
  // 9個計測していたらIDLEに戻す
  if (hitNum < 9) {
    startTimer(IGNORE_TIMER);
  } else {
    next = IDLE;
  }
}

// ===============================================

// 10ms毎に計測タイマーを加算
// BCD計算処理
void countFunc(void) {
  // 初回のカウントでブザーを鳴らして、計測を開始する
  if (preStart) {
    startBeep();
    preStart = 0;
    return;
  }

  // タイマ割り込みから呼ばれるので、呼ばれた時点では割禁状態
  if ((++countTime & 0x000f) > 0x0009) {
    countTime += 0x0006;
    if ((countTime & 0x00f0) > 0x0090) {
      countTime += 0x0060;
      if ((countTime & 0x0f00) > 0x0900) {
        countTime += 0x0600;
        if ((countTime & 0xf000) > 0x9000) {
          // 5桁目は表示しないので1から加算しない
          // ※ゼロサプレスするしないの条件になっているので5桁目が必要
          countTime = 0x10000;
        }
      }
    }
  }

  // 初弾を計測していない時は、走行中のタイムを表示する
  if (hitNum == 0) {
    setTime(countTime);
  }
}

// ランダムディレイの時間経過後に走行する
void delayFunc(void) {
  next = RUNNING;
}

// パワーダウンタイマー処理
void powerDownFunc(void) {
  next = SLEEP;
}

// ===============================================
// toXxxxx()関数は状態変更用の定義とする
//   処理時には割禁にすること

inline void toIdle(void) {
  cli();

  state = IDLE;

  disableComparator();

  // 2nd BeepでIDLEに戻った時にBeepTimerでブザーを止めるため、
  // タイマーを再開させる（既に止まっていても問題ない）
  current_timer[0] = BEEP_TIMER;
  current_timer_num = 1;

  if (hitNum > 0) hitDisp = 1;  // 1オリジン
  setTime(hitTimes[hitDisp]);   // hitTimes[0]は常に0

  // パワーダウンタイマー = 1分
  startTimer(POWERDOWN_TIMER, 6000);

  // 小数点表示
  dot = 1;

  sei();
}

inline void toRun(void) {
  cli();

  state = RUNNING;
  current_timer_num = 0;

  hitDisp = 0;
  hitNum = 0;

  countTime = 0;
  setTime(countTime);

  enableComparator();
//  startBeep();
  preStart = 1;
  startTimer(COUNT_TIMER);
  startTimer(POWERDOWN_TIMER, 30000);

  // 小数点点滅
  // 2nd Beep設定の桁点滅で点滅周期を変更しているので再設定する
  dot = 1;
  startTimer(BLINK_TIMER, 50);

  sei();
}

inline void toDelay(void) {
  cli();

  state = DELAY;
  current_timer_num = 0;

  // 鳴動中にすぐに押された場合のため
  stopBeep();

  // ランダム時間：3.5〜5.5秒
  startTimer(DELAY_TIMER, random(350, 550));

  // '----'を表示
  volatile uint8_t *ssp = segData;
  *ssp++ = ~0b00000100;
  *ssp++ = 0xff;
  *ssp++ = 0xff;
  *ssp++ = ~0b00000100;
  *ssp++ = 0xff;
  *ssp++ = 0xff;
  *ssp++ = ~0b00000100;
  *ssp++ = 0xff;
  *ssp++ = 0xff;
  *ssp++ = ~0b00000100;
  *ssp++ = 0xff;
  *ssp   = 0xff;

  // 小数点表示なし
  dot = 0;

  sei();
}

// ピン割り込みで復帰したら、割り込みを解除する
ISR(PCINT1_vect) {
  PCICR = 0;
  PCMSK1 = 0;
}

inline void toSleep(void) {
  cli();

  state = SLEEP;
  current_timer_num = 0;

  // 表示を消して、2桁目のドットだけ表示する
  // (電源ランプ無しにしたので、なにも表示しないと電源が入っているか区別がつかないため）
  // DIG-1〜4をオフ
  PORTB &= ~0b00000010;
  PORTD &= ~0b11001000;
  // DIG-2だけオン
  PORTD |=  0b01000000;

  // DPだけオン
  PORTB |= 0b00111101;
  PORTC |= 0b00000001;
  PORTD |= 0b00010100;
  PORTB &=~0b00010000;

  // start Sw / count Swのピン割り込みを設定
  PCICR  = 0b00000010;
  PCMSK1 = 0b00001100;  // PCINT11, 10

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sei();
  sleep_cpu();
  cli();
  sleep_disable();

  // ボタンの状態を更新しておく（そうしないと拾われてしまう）
  countSw = 0;

  // startSwはひろう！（スリープから直接Runをかけるため）
  // startSw = 0;

  next = IDLE;

  sei();
}

void loop() {
  // 乱数系列を適当に回しておく
  random(10);

  set_sleep_mode(SLEEP_MODE_IDLE);
  sleep_enable();
  sleep_cpu();
  sleep_disable();

  switch (state) {
    //================================================
    case IDLE:
      if (isPushed(startSw)) {
        if ((PINC >> 4) & 0x01) next = RUNNING;
        else next = DELAY;
      }
      if (isPushed(countSw)) {
        if (hitNum > 0) {
          if (++hitDisp > hitNum) {
            hitDisp = 1;
          }
          setTime(hitTimes[hitDisp]);
        }
      }
      break;

    //================================================
    case RUNNING:
      // 計測を開始して1秒以内はボタンを拾わないようにする
      cli();
      if (countTime < 0x0100) {
        sei();
        break;
      }
      sei();
      // ^^^^

      if (isPushed(startSw)) {
        if ((PINC >> 4) & 0x01) {
          state = IDLE;
          next = RUNNING;
        }
        else next = DELAY;
      }
      if (isPushed(countSw)) {
        next = IDLE;
      }
      break;

    //================================================
    case DELAY:
      if (isPushed(countSw)) {
        next = IDLE;
      }
      break;
  }

  if (state != next) {
    if (next == IDLE) {
      toIdle();
    }
    else if (next == RUNNING) {
      toRun();
    }
    else if (next == DELAY) {
      toDelay();
    }
    else if (next == SLEEP) {
      toSleep();
    }
  }
}
