#include <Arduino.h>
#include <Wire.h>
#include <LovyanGFX.hpp>
#include "ESP_I2S.h"
#include <Preferences.h>
#include <Adafruit_BME280.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <HardwareSerial.h>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cctype>
#include "esp_task_wdt.h"
#include "esp_system.h"
#if __has_include("esp_mac.h")
#include "esp_mac.h"
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <SD_MMC.h>
#include <LittleFS.h>    // tracce GPS dei voli (file KML)
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEClient.h>
#include <BLEScan.h>
#include "esp_heap_caps.h"
#include "mbedtls/md.h"      // HMAC-SHA256 per il codice di attivazione (licenza)
#include "logo.h"
#include "AirplaneBitmap.h"
#include "VarioNumbers80.h"
#include "UiFont24.h"
#include "UiFont32.h"
#include "ClockFonts.h"
#include "sunset_screen.h"
#include "TimeZoneAuto.h"   // fuso orario e ora legale automatici dalla posizione (tutto il mondo)
#include "AirfieldsAVEA.h"   // campi di volo per la pagina NEAREST FIELDS
// Elenco dei campi in uso: quello compilato nel firmware oppure, se caricato dalla pagina web WAYPOINTS
// (file XCSoar .cup), quello del file, tenuto nella PSRAM. Da qui in poi AIRFIELDS e AIRFIELD_COUNT
// indicano sempre l'elenco in uso.
static const AirfieldRec *const AIRFIELDS_FIRMWARE = AIRFIELDS;
static const int AIRFIELDS_FIRMWARE_COUNT = (int)(AIRFIELD_COUNT);
static const AirfieldRec *gAirfields = AIRFIELDS_FIRMWARE;
static int gAirfieldCount = AIRFIELDS_FIRMWARE_COUNT;
static uint32_t gAirfieldsGen = 0;   // cambia ogni volta che si carica un elenco nuovo
#ifdef AIRFIELDS
#undef AIRFIELDS
#endif
#ifdef AIRFIELD_COUNT
#undef AIRFIELD_COUNT
#endif
#define AIRFIELDS gAirfields
#define AIRFIELD_COUNT gAirfieldCount
// Direzione vera della pista dal file .cup (colonna rwdir), come scarto in gradi dal numero della pista x 10:
// 0 = non nota (elenco del firmware o file vecchio), altrimenti scarto + 20. Serve per disegnare la pista
// allineata esattamente alla rotta (il numero della pista e' arrotondato alla decina).
static uint8_t *gAirfieldHdgOfs = nullptr;

// Pacchetto BLE delle impostazioni condivise master <-> slave. Definito qui in cima perche' l'IDE Arduino
// genera i prototipi delle funzioni prima del resto del codice e alcune usano questo tipo.
#pragma pack(push, 1)
// Impostazioni condivise (tipo 11): master -> slave in notifica (1 Hz), slave -> master sulla caratteristica comandi.
// flags: bit0 quota in metri, bit1 velocita' in nodi, bit2 QFE in piedi, bit3 riferimento QFE valido.
struct BlePktPrefs {
  uint8_t type; uint8_t flags; uint8_t lubW; uint8_t lubY;
  int32_t qfeRefCm;            // quota di riferimento del QFE (RESET QFE), cm
  uint16_t qfeLow, qfeHigh;    // fascia di circuito del QFE, m
  uint16_t gotoIdx;            // GO TO verso un campo (indice in AIRFIELDS), 0xFFFF = verso casa
};
#pragma pack(pop)
#include "visibility_icons.h"

// --- Pinout Hardware ---
#define I2C_SDA     11
#define I2C_SCL     10
#define LCD_BL      5
#define TCA_ADDR    0x20  // TCA9554PWR Waveshare

#define TOUCH_ADDR  0x15
#define TOUCH_INT_PIN 4        // CST816 INT (schema 1.85C V2: TP_INT = GPIO4)
// Messaggi sulla seriale USB (monitor seriale di Arduino): 0 = spenti, 1 = accesi per una diagnosi.
// Spenti, il compilatore li toglie dal firmware (nessun tempo perso a stamparli).
#define AVEA_SERIAL_LOG 0
#if AVEA_SERIAL_LOG
#define AVEA_LOGF(...)  Serial.printf(__VA_ARGS__)
#define AVEA_LOGLN(...) Serial.println(__VA_ARGS__)
#else
#define AVEA_LOGF(...)  do { if (0) Serial.printf(__VA_ARGS__); } while (0)    // mai eseguito: il compilatore lo elimina
#define AVEA_LOGLN(...) do { if (0) Serial.println(__VA_ARGS__); } while (0)
#endif
#define TOUCH_POLL_FALLBACK_MS 150UL  // rilettura di sicurezza mentre il dito e' giu'
#define TOUCH_ALT_SDA_PIN 1  // Alcune revisioni Touch-LCD-1.85 cablano il CST816 su GPIO1/3
#define TOUCH_ALT_SCL_PIN 3
#define PCF8563_ADDR 0x51
#define BME280_I2C_ADDR 0x76

#define I2C_BUS_FREQUENCY 100000UL
#define I2C_TIMEOUT_MS 50
#define BME_MAX_CONSECUTIVE_ERRORS 5
#define BME_RECOVERY_RETRY_MS 2000UL
#define WDT_TIMEOUT_SECONDS 5
#define TOUCH_MIN_GESTURE_MS 40UL
#define SWIPE_MIN_X_PX 35
#define SWIPE_MIN_Y_PX 40
#define SWIPE_CROSS_AXIS_MAX_PX 60
#define TOUCH_MAX_X 360
#define TOUCH_MAX_Y 360

#define I2S_MCLK    2
#define I2S_BCLK    48
#define I2S_LRCK    38
#define I2S_DOUT    47
#define I2S_DIN     39
#define PA_CTRL     15
#define AUDIO_SAMPLE_RATE 24000
#define BUZZER_PIN  26

#define GPS_RX_PIN  44
#define GPS_TX_PIN  43
#define TF_SD_CLK_PIN 14
#define TF_SD_CMD_PIN 17
#define TF_SD_D0_PIN  16
#define GS_ZERO_THRESHOLD_KMH 8.0f
#ifndef DEG_TO_RAD  // gia' definita da Arduino.h
#define DEG_TO_RAD (3.14159265358979323846 / 180.0)
#endif

// --- Driver Display LovyanGFX ---
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST77916  _panel_instance;
  lgfx::Bus_SPI        _bus_instance;
  lgfx::Light_PWM      _light_instance;
public:
  LGFX() {
    {
      auto cfg = _bus_instance.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;   // 40 MHz: con 80 MHz a volte la parte alta dell'immagine arrivava corrotta
      cfg.freq_read  = 20000000;
      cfg.pin_sclk = 40;
      cfg.pin_io0  = 46;
      cfg.pin_io1  = 45;
      cfg.pin_io2  = 42;
      cfg.pin_io3  = 41;
      cfg.pin_dc   = -1;
      _bus_instance.config(cfg);
      _panel_instance.setBus(&_bus_instance);
    }
    {
      auto cfg = _panel_instance.config();
      cfg.pin_cs           = 21;
      cfg.pin_rst          = -1;
      cfg.panel_width      = 360;
      cfg.panel_height     = 360;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.invert           = true;
      cfg.rgb_order        = true;
      _panel_instance.config(cfg);
    }
    {
      auto cfg = _light_instance.config();
      cfg.pin_bl = LCD_BL;
      cfg.freq   = 1200;
      cfg.pwm_channel = 7;
      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);
    }
    setPanel(&_panel_instance);
  }
};

LGFX lcd;
// Bussola e Go To Home usano lo stesso framebuffer per evitare due allocazioni.
// da 360x360x16 bit (~253 KB) nella RAM interna.
LGFX_Sprite navigationSprite(&lcd);
bool navigationSpriteReady = false;
bool navigationSpriteAllocationFailed = false;
Adafruit_BME280 sensor_bme;

enum DisplayTheme { THEME_COLOR, THEME_WHITE_BLACK, THEME_YELLOW_BLACK };
DisplayTheme displayTheme = THEME_COLOR;   // tema in uso mentre si disegna: resta a colori fuori dalla bussola
// Tema scelto con lo swipe verticale sulla bussola: vale SOLO per la bussola (vedi CompassThemeScope),
// tutte le altre pagine restano sempre a colori.
DisplayTheme compassTheme = THEME_COLOR;
#define THEME_TEXT_SCALE (displayTheme == THEME_COLOR ? 1.0f : 1.10f)

uint16_t themeBg() {
  return displayTheme == THEME_WHITE_BLACK ? TFT_WHITE :
         (displayTheme == THEME_YELLOW_BLACK ? TFT_YELLOW : TFT_BLACK);
}
uint16_t themeColor(uint16_t color) {
  return displayTheme == THEME_COLOR ? color : TFT_BLACK;
}
void setThemeTextColor(uint16_t fg, uint16_t bg) {
  if (displayTheme == THEME_COLOR) lcd.setTextColor(fg, bg);
  else lcd.setTextColor(TFT_BLACK, themeBg());
}
// Colori "vivi": restano a colori anche nei temi bianco/nero e giallo/nero, con la tonalita'
// che resta leggibile sullo sfondo del tema. Il bianco diventa nero (illeggibile su bianco/giallo).
uint16_t themeVivid(uint16_t color) {
  if (displayTheme == THEME_COLOR) return color;
  if (color == TFT_WHITE || color == TFT_BLACK) return TFT_BLACK;
  if (color == TFT_GREEN)  return displayTheme == THEME_WHITE_BLACK ? lgfx::color565(0, 160, 0) : lgfx::color565(0, 110, 0);
  if (color == TFT_YELLOW) return displayTheme == THEME_WHITE_BLACK ? lgfx::color565(255, 140, 0) : lgfx::color565(210, 90, 0);
  if (color == TFT_CYAN)   return TFT_BLUE;
  return color;
}
void setVividTextColor(uint16_t fg) {
  lcd.setTextColor(themeVivid(fg), themeBg());
}

// AIR DATA e' sempre a colori: forza THEME_COLOR per la durata dello scope e poi ripristina il tema scelto.
struct ForceColorThemeScope {
  DisplayTheme saved;
  ForceColorThemeScope() : saved(displayTheme) { displayTheme = THEME_COLOR; }
  ~ForceColorThemeScope() { displayTheme = saved; }
};
// Bussola: applica il tema scelto con lo swipe verticale per la durata dello scope, poi torna a colori.
struct CompassThemeScope {
  DisplayTheme saved;
  CompassThemeScope() : saved(displayTheme) { displayTheme = compassTheme; }
  ~CompassThemeScope() { displayTheme = saved; }
};

Preferences preferences;
static bool sdCardReady = false;
HardwareSerial GpsSerial(1);

enum ScreenState {
  MAIN_ALT_SCREEN,
  QFE_SCREEN,
  QNH_SETUP_SCREEN,
  VARIOMETER_SCREEN,
  G_METER_SCREEN,          // G-METER rimosso: resta solo come posto della riga ROTATE DISPLAY 180 in PAGES 2/2
  COMPASS_SCREEN,
  GPS_SPEED_SCREEN,
  GPS_CLOCK_SCREEN,
  AV_SETUP_SCREEN,
  DUAL_MODE_SCREEN,
  METEO_SCREEN,
  AIR_DATA_SCREEN,
  ICT_SCREEN,
  FLIGHT_TIMER_SCREEN,
  AUTOMATIC_TIMER_SCREEN,
  FLIGHT_LOG_SCREEN,
  FLIGHT_END_SCREEN,
  FIRMWARE_SCREEN,
  FLIGHT_LOGS_VIEW_SCREEN,
  HOME_FIELD_SCREEN,
  GO_TO_HOME_SCREEN,
  GPS_INFO_SCREEN,
  ICT_ALERT_SCREEN,
  VARIO_FILTER_SCREEN,
  WIFI_LOGS_SCREEN,
  AVEA_LOGO_SCREEN,
  RISK_WARNING_SCREEN,
  SUNSET_SCREEN,
  LUBBER_LINE_SCREEN,
  COORDS_SCREEN,
  HOME_COORDS_FMT_SCREEN,  // scelta del formato coordinate (COORDS in HOME FIELD)
  HOME_COORDS_SCREEN,  // inserimento manuale delle coordinate home a rulli
  FUEL_SCREEN,         // FUEL ESTIMATE (nel giro, attivabile da PAGES 2/2)
  FUEL_SETUP_SCREEN,   // impostazioni FUEL ESTIMATE (tocco al centro)
  FUEL_RESERVE_SCREEN, // avviso RESERVE da ogni pagina
  ALT_OFFSET_SCREEN,   // taratura offset altimetro (voce OFFSET di PAGES)
  PAGE_SELECT_SCREEN,  // pagina nascosta: scelta delle pagine del giro (L sulla FIRMWARE VERSION)
  WIFI_LOGIN_SCREEN,   // tastiera: nome rete e password del WIFI LOG (pulsante CHANGE LOGIN)
  NEAREST_FIELDS_SCREEN, // i 3 campi di volo piu' vicini (nel giro dopo GO HOME)
  LICENSE_SCREEN,      // attivazione: inserimento del codice di licenza (fuori dal giro)
  AIRSPACE_ALERT_SCREEN, // avviso spazio aereo vicino / dentro (fuori dal giro)
  DENSITY_ALT_SCREEN,   // quota densita': avviso all'accensione e pagina da AIR DATA (fuori dal giro)
  FIELDS_RADAR_SCREEN,  // campi entro 30 km attorno all'aereo, rotta in alto (nel giro tra COMPASS e GO HOME)
  FIELD_SEARCH_SCREEN,  // ricerca di un campo con la tastiera (da SEARCH in NEAREST FIELDS, fuori dal giro)
  SLIP_SCREEN,          // TURN & SLIP: pallina e rateo dal MPU-6050, nel giro tra GND SPEED e GPS CLOCK
  ONLINE_WX_SCREEN      // meteo dal telefono a caratteri grandi: primo passo del giro da AIR DATA (fuori dal giro)
};

// ATTENZIONE: la scelta di PAGES 1/2 e 2/2 e' salvata come un bit per posizione in questo elenco.
// Le pagine nuove vanno aggiunte SOLO IN FONDO, cosi' dopo un aggiornamento firmware le scelte restano giuste.
// Anche i valori dell'enum ScreenState gia' esistenti non vanno spostati (pagina di avvio e DUAL salvate come numero).
const ScreenState screenOrder[] = {
  COMPASS_SCREEN,
  GO_TO_HOME_SCREEN,
  METEO_SCREEN,
  VARIOMETER_SCREEN,
  MAIN_ALT_SCREEN,
  QFE_SCREEN,
  GPS_SPEED_SCREEN,
  GPS_CLOCK_SCREEN,
  AUTOMATIC_TIMER_SCREEN,
  ICT_SCREEN,
  AV_SETUP_SCREEN,
  WIFI_LOGS_SCREEN,
  GPS_INFO_SCREEN,
  // pagina PAGES 2/2 (spente di serie)
  FLIGHT_TIMER_SCREEN,
  G_METER_SCREEN,          // G-METER rimosso: resta solo come posto della riga ROTATE DISPLAY 180 in PAGES 2/2
  FUEL_SCREEN,
  FIRMWARE_SCREEN,     // nel giro prima di GPS INFO (vedi CYCLE_ORDER); nascosta in volo reale
  NEAREST_FIELDS_SCREEN, // campi piu' vicini: nel giro dopo GO HOME, casella in PAGES 1/2
  FIELDS_RADAR_SCREEN,  // campi entro 30 km: nel giro tra COMPASS e GO HOME, casella in PAGES 2/2
  SLIP_SCREEN           // TURN & SLIP: nel giro tra GND SPEED e GPS CLOCK, casella in PAGES 3/3
};
// Fuori dal giro con lo swipe: logo (solo avvio, 5 s), MANUAL FLIGHT TIMER e FILTER
// (FILTER nascosta: il tocco sul numerico del variometro non la apre piu', filtro fisso NORMAL).
// SETUP nascosta (soglia vario fissa 1.0 m/s). G-meter disabilitato.
const int numScreens = 20;
const int PAGES_FIRST_COUNT = 13;   // pagine in PAGES 1/2; le altre (MANUAL FLIGHT TIMER, G-METER, FUEL ESTIMATE) in PAGES 2/2

// ---- Pagine del giro scelte dall'utente (pagina nascosta PAGES) ----
// Si apre con un tocco su "TAP TO PAGES" (a destra del QR) nella FIRMWARE VERSION e il PIN. La maschera (un bit per
// pagina, nello stesso ordine di screenOrder) e' salvata nel namespace NVS "pages": "Reset preferences"
// (che svuota solo "altimeter", "display" e i log voli) e il boot non lo toccano.
const char *const PAGE_NAMES[numScreens] = {
  "COMPASS", "GO HOME", "AIR DATA", "VARIO", "ALTIMETER", "QFE", "GND SPEED",
  "GPS CLOCK", "AUTO TIMER", "ICT", "SETUP", "WIFI LOG", "GPS INFO",
  "MANUAL FLIGHT TIMER", "G-METER", "FUEL ESTIMATE", "FIRMWARE VERSION", "NEAREST FIELDS", "FIELDS 30 KM", "TURN & SLIP"
};
// Ordine del giro con lo swipe (indici di screenOrder): diverso dall'ordine di memoria, cosi' una pagina
// nuova puo' stare in mezzo al giro senza spostare le scelte salvate di PAGES. FIRMWARE VERSION prima di GPS INFO.
const int CYCLE_ORDER[numScreens] = {0, 18, 1, 17, 2, 3, 4, 5, 6, 19, 7, 8, 9, 10, 11, 16, 12, 13, 14, 15};   // TURN & SLIP tra GND SPEED e GPS CLOCK   // FIELDS 30 km tra COMPASS e GO HOME, NEAREST FIELDS dopo GO HOME
const uint32_t PAGES_ALL_MASK = (uint32_t)((1UL << numScreens) - 1UL);
// Di serie (primo avvio e pagine nuove dopo un aggiornamento): tutte attive tranne G-METER. Vedi loadPagesMask().
const uint32_t PAGES_DEFAULT_MASK = (uint32_t)((1UL << PAGES_FIRST_COUNT) - 1UL);
uint32_t pagesEnabledMask = PAGES_DEFAULT_MASK;   // 32 bit: oltre 16 pagine
// FIRMWARE VERSION e' sempre attiva: da li' si raggiungono PAGES, DEMO MODE e Reset (il logo non la apre piu').
bool pageIndexEnabled(int i) {
  if (i < 0 || i >= numScreens) return false;
  if (screenOrder[i] == FIRMWARE_SCREEN) return true;
  if (screenOrder[i] == AV_SETUP_SCREEN) return false;   // SETUP nascosta: soglia vario fissa a 1.0 m/s
  return ((pagesEnabledMask >> i) & 1UL) != 0;
}
// Indice della prossima pagina attiva partendo da "from" nel verso dir (+1 / -1); se non ce n'e', resta "from".
bool realFlightActive();
// Pagine nascoste dal giro durante il volo reale (non in demo): GPS INFO, WIFI LOG, SETUP, ICT, FIRMWARE VERSION.
bool pageHiddenInFlight(int i) {
  if (i < 0 || i >= numScreens || !realFlightActive()) return false;
  const ScreenState sc = screenOrder[i];
  return sc == GPS_INFO_SCREEN || sc == WIFI_LOGS_SCREEN || sc == AV_SETUP_SCREEN || sc == ICT_SCREEN ||
         sc == FIRMWARE_SCREEN;
}
bool slipSensorFound();
bool slipPageAvailable();   // TURN & SLIP: solo con MPU-6050 trovato all'accensione (o in demo)
// Segue CYCLE_ORDER (non l'ordine di screenOrder). from / risultato = indici di screenOrder.
int nextEnabledScreenIndex(int from, int dir) {
  int pos = 0;
  for (int k = 0; k < numScreens; ++k) if (CYCLE_ORDER[k] == from) { pos = k; break; }
  for (int k = 1; k <= numScreens; ++k) {
    const int i = CYCLE_ORDER[(((pos + dir * k) % numScreens) + numScreens) % numScreens];
    if (pageIndexEnabled(i) && !pageHiddenInFlight(i) && (screenOrder[i] != SLIP_SCREEN || slipPageAvailable())) return i;
  }
  return from;
}
// DISABLE VERTICAL SWIPE (PAGES 2/2): lo swipe su/giu' non cambia piu' il tema colori.
// Salvato nel namespace "pages", quindi non toccato da "Reset preferences".
bool verticalSwipeDisabled = false;
// ROTATE DISPLAY 180 (PAGES 2/2, al posto di G-METER): display e touch ruotati di 180 gradi,
// per montare lo strumento capovolto. Salvato in "pages": non toccato da "Reset preferences".
bool displayRotated180 = false;
// LUBBER LINE (PAGES 1/2): spuntato = il tocco in alto su GO TO HOME apre le impostazioni del corridoio;
// tolto (di serie) = clic disattivato e corridoio fisso verde 1 km, giallo +1 km. Salvato in "pages".
bool lubberEditEnabled = false;
extern bool isAudioEnabled;   // AUDIO: definita piu' sotto, letta e salvata qui (non piu' in PAGES 2/2)
// ARCH BLOCK (PAGES 2/2): spuntato = le soglie sugli archi (GS di ALTIMETER, GPS SPEED, circuito QFE)
// non si spostano con il dito; lo swipe sull'arco cambia pagina come altrove. Salvato in "pages".
bool arcBlockEnabled = false;
// GS 150 (PAGES 2/2, a destra di ARCH BLOCK): spuntato = GPS SPEED e arco GS dell'ALTIMETER partono da 0-150 km/h
// (0-80 kt) invece di 0-200; oltre si allargano da soli come prima. Salvato in "pages".
bool gsScale150 = false;
// ONLY CTR (PAGES 2/3): spuntato = gli avvisi degli spazi aerei solo per le CTR (non TMA, ATZ, R, P...). Salvato in "pages".
bool aspOnlyCtr = false;
// DON'T SHOW ALERTS (PAGES 3/3): spuntato = non compare nessun avviso a tutto schermo (spazi aerei, ICT,
// ghiaccio / visibilita', tramonto, riserva carburante). LOW GS resta attivo. Salvato in "pages".
bool alertsMuted = false;
void applyDisplayRotation() {
  lcd.setRotation(displayRotated180 ? 2 : 0);   // il touch e' girato in getTouch()
}
void loadVerticalSwipeSetting() {
  if (preferences.begin("pages", true)) {
    verticalSwipeDisabled = preferences.getBool("novswipe", false);
    displayRotated180 = preferences.getBool("rot180", false);
    lubberEditEnabled = preferences.getBool("lubedit", false);
    isAudioEnabled = preferences.getBool("audio", false);   // AUDIO: di serie spento
    arcBlockEnabled = preferences.getBool("arcblock", false);
    gsScale150 = preferences.getBool("gs150", false);
    aspOnlyCtr = preferences.getBool("onlyctr", false);
    alertsMuted = preferences.getBool("noalerts", false);
    preferences.end();
  }
  audioApplyEnabled();   // amplificatore acceso solo se AUDIO e' ON
}
void saveVerticalSwipeSetting() {
  if (preferences.begin("pages", false)) {
    preferences.putBool("novswipe", verticalSwipeDisabled);
    preferences.putBool("rot180", displayRotated180);
    preferences.putBool("lubedit", lubberEditEnabled);
    preferences.putBool("audio", isAudioEnabled);
    preferences.putBool("arcblock", arcBlockEnabled);
    preferences.putBool("gs150", gsScale150);
    preferences.putBool("onlyctr", aspOnlyCtr);
    preferences.putBool("noalerts", alertsMuted);
    preferences.end();
  }
}
// Bit della pagina G-METER: l'unica che resta spenta di serie e dopo un aggiornamento firmware.
static uint32_t gMeterPageBit() {
  for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == G_METER_SCREEN) return (uint32_t)(1UL << i);
  return 0;
}
void savePagesMask() {
  if (preferences.begin("pages", false)) {
    preferences.putULong("mask32", pagesEnabledMask);   // 32 bit (prima "mask" a 16 bit)
    preferences.putUChar("count", (uint8_t)numScreens);   // quante pagine conosceva il firmware che ha salvato
    preferences.end();
  }
}
void loadPagesMask() {
  const uint32_t gBit = gMeterPageBit();
  uint32_t m = PAGES_ALL_MASK & ~gBit;   // primo avvio: tutte attive tranne G-METER
  bool haveMask = false;
  int savedCount = 0;
  if (preferences.begin("pages", true)) {
    if (preferences.isKey("mask32")) { haveMask = true; m = preferences.getULong("mask32", m); }
    else if (preferences.isKey("mask")) { haveMask = true; m = preferences.getUShort("mask", 0xFFFF); }   // firmware precedenti
    // I firmware precedenti non salvavano "count": avevano 13 pagine (PAGES 1/2)
    savedCount = preferences.getUChar("count", (uint8_t)PAGES_FIRST_COUNT);
    preferences.end();
  }
  bool changed = !haveMask;
  if (haveMask && savedCount < numScreens) {
    // Aggiornamento firmware con pagine nuove: le nuove si accendono, tranne G-METER (sempre spento).
    for (int i = savedCount; i < numScreens; ++i) m |= (uint32_t)(1UL << i);
    m &= ~gBit;
    changed = true;
    AVEA_LOGF("PAGES: aggiornamento da %d a %d pagine, nuove pagine attivate (G-METER spento)\n", savedCount, numScreens);
  }
  m &= PAGES_ALL_MASK;
  m &= ~gBit;   // G-METER non e' piu' in PAGES 2/2 (al suo posto ROTATE DISPLAY): sempre fuori dal giro
  if (m == 0) m = PAGES_ALL_MASK & ~gBit;   // almeno una pagina attiva
  pagesEnabledMask = m;
  if (changed) savePagesMask();
}
// Gesto a L: dito giu' per almeno 80 px (tratto quasi verticale), poi a destra per almeno 80 px.
__attribute__((unused)) static bool isLGestureDownRight(int startX, int startY, int cornerX, int cornerY, int endX, int endY) {
  return (cornerY - startY >= 80) && (abs(cornerX - startX) <= 50) &&
         (endX - cornerX >= 80) && (cornerY - endY <= 50);
}

ScreenState currentScreen = MAIN_ALT_SCREEN;
int currentScreenIndex = 0;
volatile bool demoMode = false;
bool demoTourActive = false;   // demo: rotazione automatica di pagine e avvisi (sketch_test)
void demoTourStart();
void demoTourStop();
void demoTourService();
bool startupSequenceActive = false;           // true mentre e' visibile il logo di avvio (5 s)
ScreenState startupTargetScreen = MAIN_ALT_SCREEN;  // pagina salvata nelle preferenze, da mostrare dopo il logo
unsigned long startupLogoEndMillis = 0;
static const unsigned long STARTUP_LOGO_DURATION_MS = 1500UL;                // durata logo avvio

// Monitoraggio rischio ghiaccio/nebbia durante il volo (dopo lo startup):
// riusa la stessa schermata di warning, ma torna alla pagina che era
// visualizzata subito prima (non a quella memorizzata per l'avvio).
// Una volta chiuso il warning, le condizioni vengono ricontrollate e
// riproposte ogni 10 minuti finche' persistono.
unsigned long lastRuntimeRiskCheckMillis = 0;
bool runtimeRiskFlightActive = false;
const unsigned long RUNTIME_RISK_CHECK_INTERVAL_MS = 10UL * 60UL * 1000UL; // 10 minuti

// Avviso tramonto: 30 minuti prima, una sola volta al giorno.
ScreenState sunsetPreviousScreen = MAIN_ALT_SCREEN;
static int sunsetShownDayKey = -1;
static unsigned long lastSunsetCheckMillis = 0;
static const unsigned long SUNSET_CHECK_INTERVAL_MS = 30000UL;   // controllo ogni 30 s
static const int SUNSET_ALERT_MINUTES_BEFORE = 30;

int ictMinutes = 15;
bool ictEnabled = false;
unsigned long ictStartMillis = 0;
ScreenState ictAlertPreviousScreen = MAIN_ALT_SCREEN;
unsigned long ictAlertStartMillis = 0;
unsigned long ictAlertLastBlinkMillis = 0;
bool ictAlertTextVisible = true;
bool ictAlertUseItalian = true;
bool ictAlertStaticDrawn = false;
bool ictAlarmActive = false;          // allarme sonoro ICT in corso (non bloccante)
int ictAlarmStep = 0;
unsigned long ictAlarmNextMillis = 0;
int ictMinutesBeforeDemo = 15;
bool ictEnabledBeforeDemo = false;
const unsigned long ICT_ALERT_DURATION_MS = 8000UL;  // durata visualizzazione allarme ICT

// Avviso rischio ghiaccio/nebbia: schermata NON bloccante (il loop continua a
// leggere GPS, timer, vario...). Si chiude con un tocco e torna alla pagina precedente.
ScreenState riskWarningPreviousScreen = MAIN_ALT_SCREEN;
// Avvisi ripetuti in volo: oltre al tocco si chiudono da soli dopo 5 secondi.
// Quello iniziale (all'accensione) resta fino al tocco.
const unsigned long RISK_WARNING_AUTO_CLOSE_MS = 5000UL;
// Piu' rischi insieme (es. CARB ICE + FOG / HAZE): le pagine si alternano ogni 3 s finche' non si tocca.
bool riskWarningCycle = false;
// All'accensione con due rischi: niente alternanza, un tocco passa alla pagina successiva (0 = nessuna)
int riskWarningStepNext = 0;
unsigned long riskWarningCycleSwitchMillis = 0;
const unsigned long RISK_WARNING_CYCLE_MS = 3000UL;
bool riskWarningAutoClose = false;
unsigned long riskWarningShownMillis = 0;
int riskWarningCarbLevel = 0;
int riskWarningFogLevel = 0;
float riskWarningTempC = NAN;          // temperatura esterna compensata, per lancetta e finestrella CARB ICE
int riskWarningDrawnTempTenth = -9998; // decimi di grado mostrati dalla lancetta nell'ultimo disegno
// Anteprima dalla pagina METEO (tocco sulla parte bassa):
// alterna AIR DATA, CARB ICE e VISIBILITY ogni 3 s; si esce con lo swipe e si torna a METEO.
bool riskPreviewActive = false;
int riskWarningForcedPage = 0;                    // 0 = automatica, 1 = CARB ICE, 2 = FOG
unsigned long riskPreviewLastSwitchMillis = 0;
// Demo: sequenza iniziale (3 s ciascuno) degli stadi VISIBILITY (good / low warning / fog),
// CARB ICE (basso / moderato / alto) e della pagina SUNSET; finita la sequenza parte la
// normale routine demo. Valori simulati coerenti con i livelli mostrati.
struct DemoRiskStage { float tempC; float humidity; int page; };   // page: 1 = CARB ICE, 2 = FOG, 3 = SUNSET
static const DemoRiskStage DEMO_RISK_STAGES[8] = {
  {25.0f, 50.0f, 2},   // GOOD VISIBILITY
  {14.5f, 61.0f, 2},   // HAZE POSSIBLE (foschia, caso Lodi: spread ~7.5 C)
  {20.0f, 80.0f, 2},   // LOW VISIBILITY WARNING
  {15.0f, 95.0f, 2},   // FOG
  {35.0f, 35.0f, 1},   // carb ice: rischio basso (verde)
  {26.0f, 65.0f, 1},   // carb ice: rischio moderato (arancione)
  {15.0f, 90.0f, 1},   // carb ice: rischio alto (rosso)
  {25.0f, 50.0f, 3}    // SUNSET
};
const int DEMO_RISK_STAGE_COUNT = 8;
const unsigned long DEMO_RISK_STAGE_MS = 3000UL;          // durata di ogni stadio
const unsigned long DEMO_INTRO_GIVEUP_MS = 5000UL;        // se un altro avviso blocca la partenza, la sequenza salta
bool demoIntroActive = false;          // true dalla partenza della demo fino alla fine della sequenza iniziale
bool demoRiskTourActive = false;
bool demoRiskTourDone = false;
int demoRiskTourStep = 0;
unsigned long demoRiskTourStepMillis = 0;
unsigned long demoIntroRequestMillis = 0;
float riskWarningHumidity = NAN;       // umidita' esterna compensata (%), mostrata nella finestrella CARB ICE
int riskWarningDrawnHumidity = -9998;  // umidita' (%) mostrata nell'ultimo disegno
bool startupRiskCheckPending = false;          // controllo iniziale, fatto dal loop()
unsigned long startupRiskCheckDeadlineMillis = 0;
extern bool bmeFoundAtBoot;
// Quanto aspettare, finito il logo, una lettura valida di temperatura e umidita' prima di
// rinunciare all'avviso. Col sensore a bordo la lettura c'e' sempre gia' pronta e questo
// tetto non si vede mai. Senza sensore (strumento slave) non arrivera' mai: aspettare tre
// secondi ritarda l'accensione per niente, e nemmeno i dati dal master possono arrivare in
// quel tempo, perche' il Bluetooth parte tre secondi dopo l'avvio.
static inline unsigned long startupRiskWaitMs() { return bmeFoundAtBoot ? 3000UL : 500UL; }

bool flightTimerRunning = false;
unsigned long flightTimerStartMillis = 0;
unsigned long flightTimerAccumMillis = 0;
bool automaticFlightRunning = false;
// Volo reale (timer automatico partito, NON in demo): blocca impostazioni e nasconde alcune pagine.
bool realFlightActive() { return automaticFlightRunning && !demoMode; }
unsigned long automaticFlightStartMillis = 0;
unsigned long automaticFlightAccumMillis = 0;
const float TAKEOFF_SPEED_START_KMH = 60.0f;
const float LANDING_SPEED_STOP_KMH = 5.0f;
const float DEMO_FLIGHT_STOP_SPEED_KMH = 60.0f;
const unsigned long SPEED_START_DEBOUNCE_MS = 3000UL;
const unsigned long SPEED_STOP_DEBOUNCE_MS = 5000UL;
unsigned long speedAboveStartSince = 0;
unsigned long speedBelowStopSince = 0;
unsigned long gpsInvalidFlightSince = 0;

// --- Bluetooth LE: condivisione GPS/sensori tra due strumenti identici ---
// MASTER = invia i propri dati (server GATT, notifiche); SLAVE = li riceve (client GATT).
// Tutto il BLE gira in un task separato (bleTask): il loop() non viene mai bloccato.
enum { BLE_ROLE_OFF = 0, BLE_ROLE_MASTER = 1, BLE_ROLE_SLAVE = 2 };
enum { BLE_ST_OFF = 0, BLE_ST_WAIT = 1, BLE_ST_LINKED = 2, BLE_ST_ERROR = 3, BLE_ST_PAUSED = 4 };
volatile uint8_t bleRole = BLE_ROLE_OFF;     // salvato in NVS ("ble_role"); se mai salvato: MASTER
volatile uint8_t bleState = BLE_ST_OFF;
volatile bool bleWifiHold = false;           // true mentre e' attivo il WiFi log: il BLE resta spento
int bleUiLastKey = -1;
void bleStartTask();
void bleCycleRole();
void bleSuspendForWifi();
void bleResumeAfterWifi();
void bleApplyRemoteGps();
void bleApplyRemoteTimer();
char flightStartUtc[16] = "--:--:--";
char flightStopUtc[16] = "--:--:--";
char automaticStartUtc[16] = "--:--:--";
char automaticStopUtc[16] = "--:--:--";
char flightDate[12] = "--/--/----";

ScreenState flightEndPreviousScreen = FLIGHT_TIMER_SCREEN;

float flightMaxAltitudeMeters = 0.0f;
float flightMaxClimbRate = 0.0f;
float flightMaxSinkRate = 0.0f;
float flightFuelUsedL = -1.0f;   // FUEL ESTIMATE del volo in corso / ultimo volo (litri, <0 = non disponibile)
float flightMaxGpsSpeedKmh = 0.0f;
float flightMinTempC = 0.0f;
float flightMaxTempC = 0.0f;
float flightMinHumidity = 0.0f;
float flightMaxHumidity = 0.0f;
float flightEndTempC = 0.0f;
float flightEndHumidity = 0.0f;
bool flightEndEnvironmentValid = false;
float flightTotalDistanceKm = 0.0f;
float flightMinG = 1.0f;
float flightMaxG = 1.0f;
bool flightEnvironmentValid = false;
bool flightAltitudeMetricValid = false;
bool flightLogMetricsStarted = false;
unsigned long flightStatsLastUpdateMillis = 0;

struct FlightLog {
  char startTime[9];
  char stopTime[9];
  char date[11];
  unsigned long durationSec;
  float maxAltitude;
  float maxClimb;
  float fuelUsed;      // FUEL ESTIMATE: litri stimati consumati nel volo (<0 = non disponibile; prima era maxSink)
  float maxSpeed;
  float distance;
  float minTemp;
  float maxTemp;
  float minHumidity;
  float maxHumidity;
  float minG;
  float maxG;
  float endTemp;
  float endHumidity;
  bool endEnvironmentValid;
};
static FlightLog bleOutgoingFlightLog = {};
static volatile bool bleLogTxPending = false;
static uint8_t bleLogTxIndex = 0;
static uint8_t bleLogTxPass = 0;
// Layout precedente del blob logs_v1: usato solo per migrare i log esistenti.
struct LegacyFlightLogV1 {
  char startTime[9];
  char stopTime[9];
  char date[11];
  unsigned long durationSec;
  float maxAltitude;
  float maxClimb;
  float maxSink;
  float maxSpeed;
  float distance;
  float minTemp;
  float maxTemp;
  float minHumidity;
  float maxHumidity;
  float minG;
  float maxG;
};
// Prototipi espliciti dopo FlightLog: impediscono all'IDE Arduino di
// generarli prima della definizione del tipo usato per riferimento.
static void sanitizeFlightLog(FlightLog &l);
static bool readLegacyLog(Preferences &prefs, int index, FlightLog &log);
static bool wifiLogEndEnvValid(const FlightLog &log);
static bool trackExistsForLog(const FlightLog &l);

const int MAX_LOGS = 10;
int currentLogViewIndex = 0;

#define SAVED_LOGS_BTN_X 110
#define SAVED_LOGS_BTN_Y 316
#define SAVED_LOGS_BTN_W 140
#define SAVED_LOGS_BTN_H 36
#define FIRMWARE_UPDATE_BTN_CENTER_X 180
#define FIRMWARE_UPDATE_BTN_CENTER_Y 250   // pulsante DEMO MODE, sotto il codice QR del manuale
#define FIRMWARE_RESET_BTN_CENTER_X 180
#define FIRMWARE_RESET_BTN_CENTER_Y 300
#define FIRMWARE_BUTTON_PADDING 3
#define FIRMWARE_RESET_PAD_X 14          // rettangolo "Reset preferences" piu' grande
#define FIRMWARE_RESET_PAD_Y 11
#define FIRMWARE_RESET_HIT_MARGIN_X 30   // area di tocco extra attorno al rettangolo
#define FIRMWARE_RESET_HIT_MARGIN_Y 10
static int firmwareUpdateButtonX = 0;
static int firmwareUpdateButtonY = 0;
static int firmwareUpdateButtonW = 0;
static int firmwareUpdateButtonH = 0;
static int firmwareResetButtonX = 0;
// Reset preferences in due clic: il primo arma (pulsante rosso "CONFIRM RESET?"), il secondo entro
// FIRMWARE_RESET_CONFIRM_MS esegue il reset. Scade da solo o uscendo dalla pagina.
#define FIRMWARE_RESET_CONFIRM_MS 5000
static unsigned long firmwareResetArmedMs = 0;   // 0 = non armato
static bool firmwareButtonAlert = false;         // disegna il prossimo pulsante in rosso
static int firmwareResetButtonY = 0;
static int firmwareResetButtonW = 0;
static int firmwareResetButtonH = 0;

bool readLogFromPrefs(int index, FlightLog &log);
int getNumSavedLogs();
void saveCurrentFlightLog();

void drawMainScreen();
void drawQfeScreen();
void updateQfeData();
bool resetQfeReference();
void drawVariometerScreen();
void drawCompassScreen(bool forceRedraw);
void drawGpsGroundSpeedScreen();
void updateGpsGroundSpeed();
void drawGpsClockScreen();
void drawAvSetupScreen();
void drawVarioFilterScreen();
void drawDualModeScreen();
void updateDualMode();
bool isDualSelected(ScreenState page);
void drawMeteoScreen();
void drawAirDataScreen();
void drawIctScreen();
void drawIctAlertScreen();
void beginIctAlertScreen();
void dismissIctAlertScreen();
void drawFlightTimerScreen();
void drawAutomaticFlightTimerScreen();
void updateAutomaticFlightTimerScreen();
void updateManualFlightTimerScreen();
void drawFlightLogScreen();
void drawFlightEndScreen();
void drawFirmwareScreen();
void otaGetMac(char *out);  // MAC WiFi del dispositivo "AA:BB:CC:DD:EE:FF" (18 byte)
void drawAveaLogoScreen();
void showCompassLogo();
void endCompassLogo();
void updateCompassLogo();
void drawFlightLogsViewScreen(int index);
void drawNoLogsViewScreen();
void drawWifiLogsScreen();
void startLogWifi();
void loadWifiLogin();
void wifiDrawChangeLoginButton();
bool wifiTapOnChangeLogin(int tx, int ty);
void openWifiLoginScreen();
void drawWifiLoginScreen();
void wifiLoginTouchRelease(bool tapValid, int diffX, int diffY, int tx, int ty);
void stopLogWifi();
void handleLogWifi();
void drawHomeFieldScreen();
void drawGoToHomeScreen();
void drawLubberLineScreen();
void drawHomeCoordsScreen();
void drawFuelScreen();
void drawFuelSetupScreen();
void loadFuelSettings();
void updateFuel();
void drawAltOffsetScreen();
void openAltOffsetScreen();
void openMeteoOffsetScreen();
void openFuelSetupScreen();
void openFuelSetupFromPages();
void updateAltOffsetScreen();
void loadAltOffset();
void drawPageSelectScreen();
void drawHomeCoordsFormatScreen();
void openHomeCoordsScreen();
void updateHomeCoordsScreen();
void applyHomeLocal(bool valid, double lat, double lon, bool redraw = true);
void bleQueueHome();
void bleQueueFuel(bool fullPressed);
void updateGoToHomeScreen();
void drawGpsInfoScreen();
void updateGpsInfoScreen();
void drawTimerGroup(const char *timer, int centerY, uint16_t fg, float scale, int bg = -1);
void updateIctTimer();
void updateFlightTimer();
void playStartupMusic();
void redrawCurrentScreen();
void drawPageLockOverlay();
void onFlightEnd();
void trackMount();
void trackStart();
void trackSample(unsigned long now, bool gpsValid, double lat, double lon, float speedKmh);
void trackFinish();
void checkAirspace();
void updateFieldsRadarScreen(bool force);
void fieldsRadarTap(int tx, int ty);
void fieldsRadarZoom(int dir);
void loadFieldsRadarZoom();
void drawDensityAltScreen();
void updateDensityAltScreen();
void dismissDensityAlt();
void openDensityAltPage();
void openOnlineWxPage();
void drawOnlineWxScreen();
bool densityAltStartupCheck();
void dismissAirspaceAlert();
void drawAirspaceAlertScreen();
bool airspaceLoad();
void loadAirspaceSettings();
void dismissFlightEndScreen();
bool startDemoMode(unsigned long nowMs);
void stopDemoMode(unsigned long nowMs);
void bleQueueCloseFlightEnd();
void bleQueueQnh(float hpa);
void bleQueueSlipZero();
void bleQueueDemo(bool on);
void bleQueueArc();
void bleQueuePrefs();
void arcApplyShared(int yelKmh, int redKmh);

// ---- Meteo dal telefono (app AVEA Link, via BLE) ----------------------------------------
// L'app scrive 10 byte sulla caratteristica BLE_MET_UUID: QNH, direzione e intensita' del
// vento, raffica. L'ora di arrivo la mette lo strumento, cosi' sa sempre quanto e' vecchio.
// Senza telefono tutto resta com'e': niente avvisi, niente attese, nessuna riga in piu'.
volatile bool metQnhValid = false;      // il telefono ha mandato un QNH
volatile bool metWindValid = false;     // il telefono ha mandato il vento
volatile float metQnh = 0.0f;           // hPa
volatile int metWindDir = 0;            // gradi, direzione DA CUI soffia
volatile float metWindKt = 0.0f;        // nodi
volatile float metGustKt = 0.0f;        // nodi
volatile unsigned long metAtMs = 0;     // millis() dell'ultimo dato ricevuto
volatile bool metVisValid = false;      // visibilita'
volatile bool metPrecipValid = false;   // pioggia
volatile bool metFrzValid = false;      // livello di congelamento
volatile bool metDewValid = false;      // punto di rugiada
volatile int metVisM = 0;               // metri
volatile float metPrecipMm = 0.0f;      // mm nell'ultima ora
volatile int metPrecipProb = 0;         // %
volatile int metFrzM = 0;               // quota dell'isoterma 0 C, metri
volatile float metDewC = 0.0f;          // gradi
// Posizione del telefono: serve SOLO a terra e solo finche' il GPS non ha il fix, per non
// lasciare vuote le pagine che hanno bisogno di una posizione. Non entra nella traccia, nel
// log, negli avvisi spazi aerei ne' nella navigazione del GO TO, e sparisce al primo fix vero.
volatile bool metPhonePosValid = false;
volatile double metPhoneLat = 0.0, metPhoneLon = 0.0;
bool phonePosFill(bool &gpsOk, double &lat, double &lon);
// Posizione del pallino con i nodi del vento nell'ultimo disegno: serve al tocco, che apre
// ONLINE WEATHER. Si azzera a ogni disegno e si riempie solo se il pallino c'e' davvero.
int windDotX = -1, windDotY = -1;
bool windDotShown = false;
int wxReturnScreen = -1;   // pagina a cui tornare con lo swipe, -1 = arrivati dal giro di AIR DATA
bool windRingTapped(int x, int y);
#define MET_FRESH_MS (30UL * 60UL * 1000UL)   // oltre mezz'ora il dato non si mostra piu'
bool metFresh();
int metAgeMin();
void drawWindArrow(LGFX_Sprite &spr, float course, uint16_t col, int rOut, int rIn);
// Quando cambia la destinazione, il meteo che abbiamo in mano e' di un altro posto: il
// telefono se ne accorge e ne chiede di nuovo, ma per dieci o venti secondi i dati vecchi
// restano gli unici che ci sono. In quei secondi il vento non si disegna e la pagina meteo
// non scrive numeri, altrimenti attribuirebbe al campo nuovo il vento del campo vecchio.
// Sullo slave il meteo non arriva dal telefono ma dal master, con la sua eta' in minuti: il dato
// e' stato ricevuto adesso ma e' vecchio di quei minuti, e la pagina deve dire l'eta' vera.
volatile unsigned long metExtraAgeMs = 0;      // eta' che il dato aveva quando e' arrivato qui
volatile bool metAlignedRemote = true;         // sullo slave: il meteo e' del campo giusto? lo dice il master
volatile unsigned long metDestChangedMs = 0;   // millis() dell'ultimo cambio di destinazione
volatile unsigned long metAlignedAtMs = 0;     // millis() del primo dato buono per il campo nuovo
volatile bool metWxAutoPending = false;        // la pagina meteo deve mostrarsi da sola
unsigned long wxAutoUntilMs = 0;               // la pagina mostrata da sola resta fino a qui
#define MET_PULSE_MS 6000UL                    // per quanto il pallino pulsa dopo il dato nuovo
#define MET_WX_AUTO_MS 4000UL                  // quanto resta la pagina che esce da sola
#define MET_TERRA_KMH 20.0f                    // sotto questa velocita' si considera a terra
bool metAligned();
bool metPulseOn();

void getLocalTimeText(char *out, size_t outSize);
bool getLocalClock(int &hour, int &minute, int &second, int &day, int &month, int &year);
void syncSoftwareClockFromGps(int hour, int minute, int second, int day, int month, int year);
void advanceSoftwareClock();
void playTone(int freqHz, int durationMs);
void handleVarioAudio();
void silenceAudio();
int drawNumericVlw(const char *text, int x, int y, decltype(MC_DATUM) datum, uint16_t fg, uint16_t bg, float scale = 1.0f, int clearMaxChars = 0);
int drawUiVlw(const char *text, int x, int y, decltype(MC_DATUM) datum, uint16_t fg, uint16_t bg, float scale = 1.0f);
void updateMeteoData();
void updateGpsClock();
void updateAirData();
void compensateOutsideAirReading(float rawTemp, float rawHumidity, float &compTemp, float &compHumidity);
extern volatile bool oatValid;     // sonda esterna DS18B20 (vedi updateOatProbe)
extern volatile bool oatFromRemote;
extern volatile float oatTempC;
extern bool meteoForceInternal;
void drawInvalidSensorMessage(const char *message);
void drawRiskWarningScreen();
void beginRiskWarningScreen(int carbRisk, int fogRisk, bool autoClose = false);
void startRiskWarning(int carbRisk, int fogRisk, bool autoClose, bool inFlight);
void dismissRiskWarningScreen();
bool bootWizardWaiting();   // accensione: SET QNH sta per aprirsi, non ridisegnare la pagina sotto
void checkRuntimeRiskWarning();
void checkSunsetAlert();
void parseNmeaSentence(const char *sentence);
void gpsSendPairCommand(const char *body);
void gpsSetRate10Hz();
void gpsConfigureNmeaMinimal();
bool nmeaHasValidChecksum(const char *s, size_t len);
void updateDemoData();
void drawThickLine(int x0, int y0, int x1, int y2, int color, int thickness);
void drawThickLineOn(lgfx::LGFXBase &g, int x0, int y0, int x1, int y1, int color, int thickness);
void drawArcPointerTri(lgfx::LGFXBase &g, int x0, int y0, int x1, int y1, int x2, int y2);
void saveScreenPreference();
void loadScreenPreference();
void saveDisplayThemePreference();
void loadDisplayThemePreference();
const char* firmwareSourceName();
void firmwareVersionString(char *out, size_t n);

volatile float seaLevelPressure = 1013.25f;
// Offset di taratura della quota BME280 (pagina OFFSET): NVS "calib"/"alt_off", fuori da "Reset preferences".
static const float ALT_OFFSET_DEFAULT_M = 11.0f;
volatile float altOffsetM = ALT_OFFSET_DEFAULT_M;
volatile float meteoTempOffsetC = 0.0f;   // METEO OFFSET (PAGES 2/2): correzione della temperatura, gradi
// Compensazione del riscaldamento interno: il termometro dentro l'ESP32 dice quanto e' caldo lo strumento.
// Temperatura vera = sensore + offset - meteoHeatK x ((chip - sensore) - meteoHeatD0), dove meteoHeatD0 e' la
// differenza chip - sensore a strumento freddo. Vale anche se lo si riaccende da caldo (il chip e' ancora caldo).
volatile float chipTempC = NAN;            // temperatura interna dell'ESP32, filtrata
float meteoHeatK = 0.0f;                   // 0 = nessuna compensazione (come prima)
float meteoHeatD0 = NAN;                   // differenza chip - sensore alla taratura a freddo
extern int pageSelectSubPage;   // PAGES 1/2 o 2/2 (definita con la pagina PAGES)
volatile unsigned long qnhChangedAtMillis = 0;
const float QNH_MIN_HPA = 800.0f;
const float QNH_MAX_HPA = 1100.0f;
const unsigned long QNH_CHANGE_GRACE_MS = 2000UL;
bool altitudeUnitsIsMeters = true;
bool qnhPendingMeters = true;   // scelta METERS/FEET in attesa: applicata solo con BACK / SAVE
ScreenState qnhReturnScreen = MAIN_ALT_SCREEN;   // pagina a cui torna BACK / SAVE della pagina SET QNH
// Accensione: dopo gli avvisi SET QNH (pulsante NEXT), poi SET LITERS, poi la pagina salvata.
bool qnhBootMode = false;            // SET QNH aperta dalla sequenza di accensione
float qnhBootStartHpa = 1013.25f;    // QNH all'apertura: lo swipe lo ripristina (salta senza salvare)
bool bootLitersWanted();             // SET LITERS dopo SET QNH? (FUEL ESTIMATE attiva in PAGES, a terra)
void bootWizardFinish();             // fine sequenza: pagina salvata nelle preferenze
float qfeReferenceAltitudeMeters = 0.0f;
bool qfeReferenceValid = false;
unsigned long qfeResetAtMillis = 0;
bool qfeStartupResetDone = false;

// [FIX RIAVVII] nmeaBuffer era una Arduino String: ogni "+=" e ogni substring()
// nel parsing NMEA alloca/libera memoria sullo heap. Con frasi GPS che arrivano
// di continuo (soprattutto tante frasi GSV quando il GPS non ha ancora il fix
// e sta cercando i satelliti) questo frammenta lo heap nel tempo fino a un
// eventuale crash. Ora e' un buffer a dimensione fissa, senza allocazioni.
const size_t NMEA_BUFFER_MAX = 120;
char nmeaBuffer[NMEA_BUFFER_MAX + 1];
size_t nmeaBufferLen = 0;
volatile bool gpsDataValid = false;
volatile bool gpsRmcStatusValid = false;
volatile float gps_latitude = 0.0f;
volatile float gps_longitude = 0.0f;
volatile float gps_altitude = 0.0f;
volatile float gps_ground_speed_kmh = 0.0f;
volatile float gps_course = 0.0f;
volatile int gps_satellites = 0;
volatile int gps_fix_quality = 0;
volatile unsigned long lastGpsMessageMillis = 0;
volatile unsigned long lastRmcMillis = 0;
volatile unsigned long lastGgaMillis = 0;
volatile unsigned long lastOwnFixMillis = 0;  // ultimo fix valido del GPS di questo strumento (non quello ricevuto via BLE)
volatile int gps_utc_hours = 0;
volatile int gps_utc_minutes = 0;
volatile int gps_utc_seconds = 0;
volatile int gps_utc_day = 0;
volatile int gps_utc_month = 0;
volatile int gps_utc_year = 0;

// --- Info GPS estese (schermata GPS INFO) ---
volatile int gps_fix_type = 0;
volatile bool gps_has_gps = false;
volatile bool gps_has_glonass = false;
volatile bool gps_has_galileo = false;
volatile bool gps_has_bds = false;
volatile bool gps_has_qzss = false;
volatile unsigned long gps_last_gsv_gps     = 0;
volatile unsigned long gps_last_gsv_glonass = 0;
volatile unsigned long gps_last_gsv_galileo = 0;
volatile unsigned long gps_last_gsv_bds     = 0;
volatile unsigned long gps_last_gsv_qzss    = 0;
const unsigned long GSV_TIMEOUT_MS = 3000; // costellazione "persa" se nessuna GSV entro questo tempo

// --- Cache per redraw senza sfarfallio della schermata GPS INFO ---
static char     gpsInfoLastFix[12]        = "";
static uint16_t gpsInfoLastFixColor       = 0xFFFF;
static char     gpsInfoLastQuality[24]    = "";
static int      gpsInfoLastSats           = -1;
static bool     gpsInfoLastSatsValid      = false;
static bool     gpsInfoLastHasGps         = false;
static bool     gpsInfoLastHasGlonass     = false;
static bool     gpsInfoLastHasGalileo     = false;
static bool     gpsInfoLastHasBds         = false;
static bool     gpsInfoLastHasQzss        = false;
static bool     gpsInfoConstInit          = false;
static int      gpsInfoLastUtcH           = -1;
static int      gpsInfoLastUtcM           = -1;
static int      gpsInfoLastUtcS           = -1;
static bool     gpsInfoLastTimeValid      = false;

static const int GPS_INFO_VALUE_RIGHT_X = 330;

volatile float meteo_temperature_c = 0.0f;
volatile float meteo_humidity_percent = 0.0f;
volatile bool meteoValid = false;



class SimpleKalmanFilter {
public:
  float _q, _r, _x_last, _p_last, _k;
  SimpleKalmanFilter(float q, float r, float init_val) {
    _q = q; _r = r; _x_last = init_val; _p_last = 1.0f;
  }
  float update(float measurement) {
    float p_temp = _p_last + _q;
    _k = p_temp / (p_temp + _r);
    float x_current = _x_last + _k * (measurement - _x_last);
    _p_last = (1.0f - _k) * p_temp;
    _x_last = x_current;
    return x_current;
  }
  void setParams(float q, float r) { _q = q; _r = r; }
};

SemaphoreHandle_t sensorDataMutex;
SemaphoreHandle_t i2cMutex;
bool bmeReady = false;
bool bmeFoundAtBoot = false;   // false = strumento senza sensori: niente recupero I2C (resettava il bus e uccideva il touch)
uint8_t bmeConsecutiveErrors = 0;
unsigned long lastBmeRecoveryAttemptMillis = 0;
unsigned long lastBmeValidMillis = 0;
const unsigned long BME_DATA_TIMEOUT_MS = 1000UL;
const float BME_MIN_PRESSURE_HPA = 300.0f;
const float BME_MAX_PRESSURE_HPA = 1100.0f;
// [FIX 2] Era 200.0f: tollerava di fatto 2000 m/s su 100 ms.
// 10 m su 100 ms = 100 m/s di vario estremo, rete di sicurezza reale.
const float BME_MAX_ALTITUDE_STEP_M = 10.0f;
bool sharedAltitudeValid = false;
bool sharedVarioValid = false;
float sharedFilteredAlt = 0.0f;
float sharedVarioKalman = 0.0f;
float maxClimbRate = 0.0f;
float maxSinkRate = 0.0f;

volatile float rawAltitudeShared = 0.0f;
volatile bool rawAltitudeValid = false;
volatile bool varioResetRequested = false;
// [FIX 1] Flag impostato dal sensorTask ad ogni nuova lettura BME280 (10 Hz).
volatile bool newAltitudeSample = false;
// [FIX A] Ultima lettura grezza accettata dal BME280. Usata per il gradino
// massimo tra due letture consecutive, cosi' il check non dipende dallo
// stato (momentaneamente indietro) del filtro di Kalman.
float lastAcceptedRawAlt = NAN;
static bool offsetChangedReset = false;   // sensorTask: offset cambiato, reset del vario alla prossima lettura

float alt[51];
double tim[51];  // millisecondi: double, un float perde risoluzione dopo ~4.6 ore
float vario = 0;
float vario_kalman = 0;
// Il vario viene calcolato solo sui nuovi campioni BME a 10 Hz: tre punti
// limitano la finestra della regressione a circa 0.2 s.
const unsigned char vario_samples = 3;
const unsigned char vario_maxsamples = 50;
float kalman_alt = 0;
float kalman_P_alt = 1;
const float kalman_Q_alt = 0.15f;
const float kalman_R_alt = 0.3f;
float kalman_vario = 0;
float kalman_P_vario = 1;
enum VarioFilterLevel { FILTER_SMOOTH, FILTER_NORMAL, FILTER_REACTIVE };
VarioFilterLevel varioFilterLevel = FILTER_NORMAL;
float kalman_Q_vario = 0.005f;
float kalman_R_vario = 10.0f;
bool varioBufferInitialized = false;

// Usa int nel parametro per evitare che l'auto-generazione dei prototipi .ino
// dichiari la funzione prima dell'enum VarioFilterLevel.
void setVarioFilterLevel(int level) {
  varioFilterLevel = (VarioFilterLevel)level;
  switch (varioFilterLevel) {
    case FILTER_SMOOTH:   kalman_Q_vario = 0.15f; kalman_R_vario = 4.0f; break;
    case FILTER_NORMAL:   kalman_Q_vario = 0.18f; kalman_R_vario = 4.0f; break;
    case FILTER_REACTIVE: kalman_Q_vario = 0.40f; kalman_R_vario = 3.0f; break;
  }
}

uint16_t touchX = 0, touchY = 0;
uint16_t touchStartX = 0, touchStartY = 0;
uint16_t touchCornerX = 0, touchCornerY = 0;   // punto piu' in basso del gesto (angolo della L)
bool isTouching = false;
// ---- Blocco pagina (in volo): dito premuto 2 s su una pagina del giro = blocca / sblocca ----
// Bloccata: swipe e tocchi ignorati; lucchetto rosso lampeggiante in basso a destra, vicino alla cornice.
// Con il blocco inserito NON compare nessun avviso (spazi aerei, riserva, ICT, ghiaccio / visibilita', tramonto):
// quelli ancora validi compaiono appena si sblocca. Solo su questo strumento; alla riaccensione la pagina e' sbloccata.
bool pageLocked = false;
static unsigned long pageLockSplashUntil = 0;   // lucchetto grande al centro per 1 s quando si blocca
static bool pageLockSplashOn = false;
static unsigned long pageLockLastDrawMs = 0;
uint8_t touchConsecutiveDownSamples = 0;
bool touchGestureArmed = false;
unsigned long lastTouchMillis = 0;
unsigned long touchStartAtMillis = 0;
bool ignoreTouchUntilRelease = false;
TwoWire *touchI2cBus = &Wire;
bool touchI2cReady = false;
void configureCST816NoSleep();  // definita dopo initTouchI2cBus()
volatile bool touchIntFlag = false;
volatile uint32_t touchIntCount = 0;
bool touchSkippedNoInt = false;  // getTouch: nessun INT nuovo, non e' un errore I2C
void IRAM_ATTR onTouchInt() { touchIntFlag = true; touchIntCount++; }
int maxTouchTravel = 0;

int lastPrintedAlt = -99999;
bool mainAltitudeErrorDisplayed = false;
int lastPrintedQfeMeters = -99999;
int lastPrintedQfeFeet = -99999;
char lastQfeInvalidMessage[20] = "";
const char* lastPrintedUnit = "";
float lastPrintedVario = -99999.0f;
int lastVarioDirection = 99;
// Quota mostrata sotto "m/s" nella pagina VARIOMETER
const int VARIO_ALT_FORCE_REDRAW = 2147483647;
const int VARIO_ALT_INVALID = -2147483647;
int lastPrintedVarioAlt = VARIO_ALT_FORCE_REDRAW;
bool lastPrintedVarioAltMeters = true;
// Barre variometro sulla schermata altimetro (lato sinistro)
int lastMainVarioTenth = 99999;   // ultimo valore intero mostrato (m/s); 99999 = da ridisegnare, -99998 = non valido
int lastMainVarioTextW = 0;       // larghezza in px dell'ultimo numero disegnato (per cancellarlo)
int lastMainAltCenterX = 180;     // centro orizzontale con cui e' stato disegnato il numero della quota
float lastMainAltScale = 1.14f;     // scala con cui e' stato disegnato il numero della quota
int lastMainVarioSign = 99;
int lastMainVarioLit = -1;
float maxGpsSpeedKmh = 0.0f;

bool isAudioEnabled = false;   // AUDIO (PAGES 2/2): di serie spento
bool isLightMode = true;
bool dualModeEnabled = false;
ScreenState dualSelection[2] = { MAIN_ALT_SCREEN, VARIOMETER_SCREEN };
int dualSelectionCount = 0;
int dualActiveIndex = 0;
unsigned long dualLastSwitchMillis = 0;
const unsigned long DUAL_SWITCH_INTERVAL_MS = 3000UL;
bool buzzEnabled = true;
bool waveMode = false;
int avThresholdIndex = 3;  // default 1.0 m/s (indice in avLiftThresholds)
const float avLiftThresholds[4] = {0.2f, 0.5f, 0.8f, 1.0f};
float varioUpThreshold = avLiftThresholds[3];
float varioDownThreshold = -avLiftThresholds[3];
const float VARIO_EMERGENCE_THRESHOLD = -5.1f;
const float WAVE_CLIMB_OFFSET = 0.2f;
const float WAVE_SINK_THRESHOLD = -0.5f;
int audioVolumePercent = 70;
unsigned long lastVarioBeepMillis = 0;
unsigned long lastVarioBuzzMillis = 0;
int lastVarioAudioFreq = 0;
bool varioAudioActive = false;
int sinkToneActiveFreq = 700;
int lastPrintedMeteoTemp = -99999;
int lastPrintedMeteoHumidity = -99999;
int lastPrintedAirDewPoint = -99999;
int lastPrintedAirCloudBase = -99999;
int lastPrintedAirDensityAlt = -99999;
unsigned long lastAirDataRedrawMillis = 0;
const unsigned long AIR_DATA_REDRAW_INTERVAL_MS = 250;

float lastPrintedCompassCourse = -9999.0f;
float lastPrintedCompassSpeed = -9999.0f;
bool lastGpsValid = false;
// timezoneIndex resta solo per compatibilita' del pacchetto BLE: il fuso ora e' automatico (TimeZoneAuto.h).
int timezoneIndex = 0;
// ---- Fuso orario automatico: fuso e ora legale calcolati dalla posizione, anche all'estero ----
// Posizione usata: GPS valido, altrimenti la home, altrimenti l'ultima salvata (NVS "tz_lat"/"tz_lon"),
// altrimenti Milano. Il calcolo segue le regole IANA di ogni paese (ora legale compresa).
double tzAutoLat = 45.4642, tzAutoLon = 9.19;
int tzAutoRule = -1;               // zona in uso (la posizione si salva solo quando cambia zona)
int32_t tzAutoOffsetSec = 3600;    // ultimo scostamento dall'UTC calcolato (secondi)
bool tzAutoIsDst = false;          // ora legale attiva
char tzAutoAbbr[12] = "CET";       // sigla della zona (CET, CEST, BST, UTC+3...)
uint32_t lastPrintedClockKey = 0xFFFFFFFFUL;
bool gpsClockHasValidTime = false;
bool softwareClockInitialized = false;
int softwareClockHour = 0, softwareClockMinute = 0, softwareClockSecond = 0;
int softwareClockDay = 1, softwareClockMonth = 1, softwareClockYear = 2000;
unsigned long softwareClockTickMillis = 0;
unsigned long softwareClockLastGpsSyncMillis = 0;
const unsigned long GPS_CLOCK_FRESH_MS = 3000UL;
int lastSunDayKey = -1;
int lastSunUtcOffset = 99;
float lastSunLat = NAN;
float lastSunLon = NAN;
char sunriseText[8] = "--:--";
char sunsetText[8] = "--:--";
bool sunTimesValid = false;
bool homePositionValid = false;
// Formato delle coordinate scelto nella pagina COORD FORMAT: vale per i rulli e per HOME FIELD (NVS "hc_fmt").
enum { HC_FMT_XCSOAR = 0, HC_FMT_GARMIN = 1, HC_FMT_SKYDEMON = 2, HC_FMT_GOOGLE = 3, HC_FMT_COUNT = 4 };
static const char *const HC_FMT_NAMES[HC_FMT_COUNT] = { "XCSOAR", "GARMIN", "SKY DEMON", "GOOGLE MAPS" };
int hcFormat = HC_FMT_GARMIN;
double homeLatitude = 0.0;
double homeLongitude = 0.0;

enum HomeConfirmAction { HOME_CONFIRM_NONE, HOME_CONFIRM_SET, HOME_CONFIRM_RESET };
HomeConfirmAction homeConfirmPending = HOME_CONFIRM_NONE;
unsigned long homeConfirmRequestedAt = 0;
const unsigned long HOME_CONFIRM_TIMEOUT_MS = 5000UL;

bool goToHomeNeedsRedraw = true;
int lastGoToHomeGpsState = -1;
int lastGoToHomeXteState = -1;
int lastGoToHomeArrowStage = -1;   // 6 freccia fissa, 7/8 freccia lampeggiante, 9 bersaglio rosso, 10 +giallo, 11/12 +verde lampeggiante, 13 +verde fisso
// Corridoio fuori rotta (linee lubber di GO TO HOME), impostabile nella pagina LUBBER LINE.
const float LUBBER_WIDTH_OPTIONS_KM[4] = {1.0f, 2.0f, 3.0f, 5.0f};   // verde: entro questa distanza per lato
const float LUBBER_YELLOW_OPTIONS_KM[3] = {1.0f, 2.0f, 3.0f};        // giallo: oltre il verde, fino a +questa distanza
const double LUBBER_MIN_LEG_KM = 0.5;                                 // sotto questa tratta il fuori rotta non e' definito
int lubberWidthIdx = 0;    // 1 km
int lubberYellowIdx = 0;   // +1 km
// Tratta di riferimento: dal punto di decollo (o primo fix con GO TO HOME aperta) fino a casa.
bool lubberLegValid = false;
// Corridoio attivo solo se ci si avvicina a casa: spento oltre 100 gradi tra rotta e direzione casa,
// riacceso sotto 80 (isteresi). Sotto LUBBER_MIN_SPEED_KMH la rotta GPS non e' affidabile: stato invariato.
const double LUBBER_OFF_DEG = 100.0;
const double LUBBER_ON_DEG = 80.0;
const float LUBBER_MIN_SPEED_KMH = 10.0f;
const double LUBBER_REANCHOR_DEG = 20.0;   // rotta entro 20 gradi da casa: la tratta riparte da qui
bool lubberActive = true;
// Imbuto automatico verso casa: sotto LUBBER_FUNNEL_REF_KM il corridoio si stringe in proporzione.
const double LUBBER_FUNNEL_REF_KM = 10.0;
const float LUBBER_MIN_GREEN_KM = 0.5f;    // larghezza verde minima per lato
const float LUBBER_MIN_YELLOW_KM = 0.3f;   // fascia gialla minima oltre il verde
double lubberLegLat = 0.0, lubberLegLon = 0.0;
// ---- Destinazione di GO HOME: la home oppure un campo scelto in NEAREST FIELDS (GO TO) ----
// La home salvata non cambia: GO TO e' solo temporaneo (non sopravvive allo spegnimento).
bool navGotoActive = false;
int navGotoIdx = -1;
double navGotoLat = 0.0, navGotoLon = 0.0;
static inline bool navTargetValid() { return navGotoActive || homePositionValid; }
static inline double navTargetLat() { return navGotoActive ? navGotoLat : homeLatitude; }
static inline double navTargetLon() { return navGotoActive ? navGotoLon : homeLongitude; }
void navSetGoto(int idx) {
  if (idx < 0 || idx >= AIRFIELD_COUNT) return;
  navGotoActive = true;
  navGotoIdx = idx;
  navGotoLat = AIRFIELDS[idx].lat5 / 100000.0;
  navGotoLon = AIRFIELDS[idx].lon5 / 100000.0;
  lubberLegValid = false;   // il corridoio riparte da qui verso il campo
  goToHomeNeedsRedraw = true;
}
void navCancelGoto() {
  if (!navGotoActive) return;
  navGotoActive = false;
  navGotoIdx = -1;
  lubberLegValid = false;
  goToHomeNeedsRedraw = true;
}
static double lubberAngDist(double la1, double lo1, double la2, double lo2) {
  const double dLa = la2 - la1, dLo = lo2 - lo1;
  const double a = sin(dLa / 2.0) * sin(dLa / 2.0) + cos(la1) * cos(la2) * sin(dLo / 2.0) * sin(dLo / 2.0);
  return 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}
static double lubberBearing(double la1, double lo1, double la2, double lo2) {
  const double dLo = lo2 - lo1;
  return atan2(sin(dLo) * cos(la2), cos(la1) * sin(la2) - sin(la1) * cos(la2) * cos(dLo));
}

// Avvisi rimandati vicino alla meta (campo del GO TO o home), solo in volo (timer automatico partito):
// entrando entro 5 km (come la vista pista) gli avvisi (ICT, spazi aerei, ghiaccio / visibilita', tramonto,
// riserva carburante) non compaiono; quelli ancora validi compaiono appena ci si allontana oltre 5.5 km.
// LOW GS resta sempre attivo.
// Senza GPS lo stato resta com'era.
extern bool automaticFlightRunning;
static const double ALERT_DEFER_IN_KM = 5.0, ALERT_DEFER_OUT_KM = 5.5;
static bool alertDeferActive = false;
extern bool pageLocked;
extern bool alertsMuted;
bool alertsDeferred() { return alertsMuted || pageLocked || demoTourActive || (alertDeferActive && automaticFlightRunning); }   // DON'T SHOW ALERTS o pagina bloccata: nessun avviso
void updateAlertDeferral() {
  static unsigned long lastMs = 0;
  if (millis() - lastMs < 500UL) return;
  lastMs = millis();
  bool ok = false; double lat = 0.0, lon = 0.0;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)2) == pdTRUE) {
    ok = gpsDataValid; lat = gps_latitude; lon = gps_longitude;
    xSemaphoreGive(sensorDataMutex);
  } else return;
  if (!navTargetValid()) { alertDeferActive = false; return; }
  if (!ok || !isfinite(lat) || !isfinite(lon)) return;
  const double km = lubberAngDist(lat * DEG_TO_RAD, lon * DEG_TO_RAD,
                                  navTargetLat() * DEG_TO_RAD, navTargetLon() * DEG_TO_RAD) * 6371.0;
  if (!alertDeferActive && km <= ALERT_DEFER_IN_KM) alertDeferActive = true;
  else if (alertDeferActive && km > ALERT_DEFER_OUT_KM) alertDeferActive = false;
}
// Fuori rotta in km, positivo = a destra della tratta (guardando verso casa).
static bool lubberCrossTrackKm(double lat, double lon, float &xteKm) {
  if (!lubberLegValid || !navTargetValid()) return false;
  const double R = 6371.0;
  const double la1 = lubberLegLat * DEG_TO_RAD, lo1 = lubberLegLon * DEG_TO_RAD;
  const double la2 = navTargetLat() * DEG_TO_RAD, lo2 = navTargetLon() * DEG_TO_RAD;
  const double la3 = lat * DEG_TO_RAD, lo3 = lon * DEG_TO_RAD;
  if (lubberAngDist(la1, lo1, la2, lo2) * R < LUBBER_MIN_LEG_KM) return false;
  const double d13 = lubberAngDist(la1, lo1, la3, lo3);
  double sx = sin(d13) * sin(lubberBearing(la1, lo1, la3, lo3) - lubberBearing(la1, lo1, la2, lo2));
  if (sx > 1.0) sx = 1.0;
  if (sx < -1.0) sx = -1.0;
  xteKm = (float)(asin(sx) * R);
  return true;
}
// Se il volo e' partito da casa (tratta decollo->casa troppo corta) il corridoio non sarebbe mai
// definito: appena ci si allontana da casa di almeno LUBBER_REANCHOR_KM la tratta viene
// riancorata alla posizione attuale (corridoio = rotta da qui verso casa).
const double LUBBER_REANCHOR_KM = 2.0;
static void lubberReanchorIfNeeded(double lat, double lon) {
  if (!lubberLegValid || !navTargetValid()) return;
  const double R = 6371.0;
  const double la2 = navTargetLat() * DEG_TO_RAD, lo2 = navTargetLon() * DEG_TO_RAD;
  const double legKm = lubberAngDist(lubberLegLat * DEG_TO_RAD, lubberLegLon * DEG_TO_RAD, la2, lo2) * R;
  if (legKm >= LUBBER_REANCHOR_KM) return;
  const double nowKm = lubberAngDist(lat * DEG_TO_RAD, lon * DEG_TO_RAD, la2, lo2) * R;
  if (nowKm >= LUBBER_REANCHOR_KM) { lubberLegLat = lat; lubberLegLon = lon; }
}
int lastGoToHomeBearingDeg = -999;
int lastGoToHomeCourseDeg = -999;
int lastGoToHomeDistanceTenth = -999;
long lastGoToHomeEtaSeconds = -999;
unsigned long lastGoToHomeRedrawMillis = 0;
const unsigned long GO_TO_HOME_REDRAW_INTERVAL_MS = 200UL;

// --- Archiviazione log voli ---
// Tutti i log stanno in un unico blob NVS (attualmente "logs_v2") piu' un contatore, con una
// copia in RAM (flightLogCache): a fine volo c'e' UNA sola scrittura NVS invece
// di ~150 chiavi. Se la struct FlightLog cambia dimensione il blob non viene piu'
// riconosciuto: in quel caso usa un nuovo nome di chiave (es. "logs_v2").
static const char *FLIGHT_LOG_NS = "flightlogs";
static const char *FLIGHT_LOG_BLOB_KEY = "logs_v2";
static const char *FLIGHT_LOG_BLOB_V1_KEY = "logs_v1";
static FlightLog flightLogCache[MAX_LOGS];
static int flightLogCacheCount = 0;
static bool flightLogCacheLoaded = false;

static void sanitizeFlightLog(FlightLog &l) {
  l.startTime[sizeof(l.startTime) - 1] = '\0';
  l.stopTime[sizeof(l.stopTime) - 1] = '\0';
  l.date[sizeof(l.date) - 1] = '\0';
  if (!isfinite(l.endTemp) || !isfinite(l.endHumidity) ||
      l.endHumidity < 0.0f || l.endHumidity > 100.0f) {
    l.endEnvironmentValid = false;
  }
}

// Vecchio formato (una chiave NVS per campo): serve solo per migrare i log gia' salvati.
static const char *const LEGACY_LOG_SUFFIXES[] = {
  "start", "stop", "date", "dur", "maxAlt", "maxClimb", "maxSink", "maxSpeed",
  "dist", "minTemp", "maxTemp", "minHum", "maxHum", "minG", "maxG"
};

static bool readLegacyLog(Preferences &prefs, int index, FlightLog &log) {
  memset(&log, 0, sizeof(log));
  char key[16];
  auto k = [&](const char *suffix) -> const char * {
    snprintf(key, sizeof(key), "l%d_%s", index, suffix);
    return key;
  };
  String s = prefs.getString(k("start"), "");
  if (s.length() != sizeof(log.startTime) - 1) return false;
  strncpy(log.startTime, s.c_str(), sizeof(log.startTime) - 1);
  log.startTime[sizeof(log.startTime) - 1] = '\0';
  s = prefs.getString(k("stop"), "");
  if (s.length() != sizeof(log.stopTime) - 1) return false;
  strncpy(log.stopTime, s.c_str(), sizeof(log.stopTime) - 1);
  log.stopTime[sizeof(log.stopTime) - 1] = '\0';
  s = prefs.getString(k("date"), "");
  if (s.length() != sizeof(log.date) - 1) return false;
  strncpy(log.date, s.c_str(), sizeof(log.date) - 1);
  log.date[sizeof(log.date) - 1] = '\0';
  log.durationSec = prefs.getUInt(k("dur"), 0);
  log.maxAltitude = prefs.getFloat(k("maxAlt"), 0.0f);
  log.maxClimb = prefs.getFloat(k("maxClimb"), 0.0f);
  log.fuelUsed = -1.0f;   // i log vecchi non hanno la stima carburante
  log.maxSpeed = prefs.getFloat(k("maxSpeed"), 0.0f);
  log.distance = prefs.getFloat(k("dist"), 0.0f);
  log.minTemp = prefs.getFloat(k("minTemp"), 0.0f);
  log.maxTemp = prefs.getFloat(k("maxTemp"), 0.0f);
  log.minHumidity = prefs.getFloat(k("minHum"), 0.0f);
  log.maxHumidity = prefs.getFloat(k("maxHum"), 0.0f);
  log.minG = prefs.getFloat(k("minG"), 1.0f);
  log.maxG = prefs.getFloat(k("maxG"), 1.0f);
  log.endEnvironmentValid = false;
  return true;
}

static bool saveFlightLogCache() {
  Preferences prefs;
  if (!prefs.begin(FLIGHT_LOG_NS, false)) {
    AVEA_LOGLN("FlightLog: apertura NVS in scrittura fallita");
    return false;
  }
  const size_t written = prefs.putBytes(FLIGHT_LOG_BLOB_KEY, flightLogCache, sizeof(flightLogCache));
  prefs.putInt("log_count", flightLogCacheCount);
  prefs.end();
  if (written != sizeof(flightLogCache)) {
    AVEA_LOGLN("FlightLog: scrittura blob NVS incompleta");
    return false;
  }
  return true;
}

static void loadFlightLogCache() {
  if (flightLogCacheLoaded) return;
  flightLogCacheLoaded = true;
  flightLogCacheCount = 0;
  memset(flightLogCache, 0, sizeof(flightLogCache));

  Preferences prefs;
  if (!prefs.begin(FLIGHT_LOG_NS, false)) {
    AVEA_LOGLN("FlightLog: apertura NVS fallita");
    return;
  }
  const int storedCount = constrain(prefs.getInt("log_count", 0), 0, MAX_LOGS);
  const size_t blobLen = prefs.getBytesLength(FLIGHT_LOG_BLOB_KEY);
  const size_t legacyBlobLen = prefs.getBytesLength(FLIGHT_LOG_BLOB_V1_KEY);

  if (blobLen == sizeof(flightLogCache)) {
    prefs.getBytes(FLIGHT_LOG_BLOB_KEY, flightLogCache, sizeof(flightLogCache));
    for (int i = 0; i < MAX_LOGS; ++i) sanitizeFlightLog(flightLogCache[i]);
    flightLogCacheCount = storedCount;
  } else if (blobLen != 0) {
    AVEA_LOGLN("FlightLog: blob logs_v2 NVS con dimensione diversa, log ignorati");
  } else if (legacyBlobLen == sizeof(LegacyFlightLogV1) * MAX_LOGS) {
    LegacyFlightLogV1 legacyCache[MAX_LOGS];
    memset(legacyCache, 0, sizeof(legacyCache));
    if (prefs.getBytes(FLIGHT_LOG_BLOB_V1_KEY, legacyCache, sizeof(legacyCache)) == sizeof(legacyCache)) {
      for (int i = 0; i < MAX_LOGS; ++i) {
        const LegacyFlightLogV1 &oldLog = legacyCache[i];
        FlightLog &newLog = flightLogCache[i];
        memcpy(newLog.startTime, oldLog.startTime, sizeof(newLog.startTime));
        memcpy(newLog.stopTime, oldLog.stopTime, sizeof(newLog.stopTime));
        memcpy(newLog.date, oldLog.date, sizeof(newLog.date));
        newLog.durationSec = oldLog.durationSec;
        newLog.maxAltitude = oldLog.maxAltitude;
        newLog.maxClimb = oldLog.maxClimb;
        newLog.fuelUsed = -1.0f;   // i log vecchi non hanno la stima carburante
        newLog.maxSpeed = oldLog.maxSpeed;
        newLog.distance = oldLog.distance;
        newLog.minTemp = oldLog.minTemp;
        newLog.maxTemp = oldLog.maxTemp;
        newLog.minHumidity = oldLog.minHumidity;
        newLog.maxHumidity = oldLog.maxHumidity;
        newLog.minG = oldLog.minG;
        newLog.maxG = oldLog.maxG;
        newLog.endTemp = 0.0f;
        newLog.endHumidity = 0.0f;
        newLog.endEnvironmentValid = false;
        sanitizeFlightLog(newLog);
      }
      flightLogCacheCount = storedCount;
      const size_t written = prefs.putBytes(FLIGHT_LOG_BLOB_KEY, flightLogCache, sizeof(flightLogCache));
      prefs.putInt("log_count", flightLogCacheCount);
      if (written == sizeof(flightLogCache)) {
        prefs.remove(FLIGHT_LOG_BLOB_V1_KEY);
        AVEA_LOGLN("FlightLog: log migrati da logs_v1 a logs_v2");
      } else {
        AVEA_LOGLN("FlightLog: migrazione logs_v1 non riuscita; log conservati in RAM");
      }
    }
  } else if (legacyBlobLen != 0) {
    AVEA_LOGLN("FlightLog: blob logs_v1 NVS con dimensione diversa, log ignorati");
  } else if (storedCount > 0) {
    // Migrazione dal vecchio formato a chiavi multiple.
    int migrated = 0;
    for (int i = 0; i < storedCount; ++i) {
      if (readLegacyLog(prefs, i, flightLogCache[migrated])) ++migrated;
    }
    flightLogCacheCount = migrated;
    if (migrated > 0) {
      const size_t written = prefs.putBytes(FLIGHT_LOG_BLOB_KEY, flightLogCache, sizeof(flightLogCache));
      prefs.putInt("log_count", migrated);
      if (written == sizeof(flightLogCache)) {
        // Blob scritto: si possono togliere le vecchie chiavi (liberano spazio NVS).
        for (int i = 0; i < MAX_LOGS; ++i) {
          for (size_t f = 0; f < sizeof(LEGACY_LOG_SUFFIXES) / sizeof(LEGACY_LOG_SUFFIXES[0]); ++f) {
            char key[16];
            snprintf(key, sizeof(key), "l%d_%s", i, LEGACY_LOG_SUFFIXES[f]);
            prefs.remove(key);
          }
        }
        AVEA_LOGF("FlightLog: migrati %d log al nuovo formato\n", migrated);
      }
    }
  }
  prefs.end();
}

int getNumSavedLogs() {
  loadFlightLogCache();
  return flightLogCacheCount;
}

bool readLogFromPrefs(int index, FlightLog &log) {
  loadFlightLogCache();
  if (index < 0 || index >= flightLogCacheCount) return false;
  log = flightLogCache[index];
  return true;
}

void saveCurrentFlightLog() {
  FlightLog log;
  memset(&log, 0, sizeof(log));
  strncpy(log.startTime, automaticStartUtc, sizeof(log.startTime)-1);
  log.startTime[sizeof(log.startTime)-1] = '\0';
  strncpy(log.stopTime, automaticStopUtc, sizeof(log.stopTime)-1);
  log.stopTime[sizeof(log.stopTime)-1] = '\0';
  if (strcmp(flightDate, "--/--/----") != 0) {
    strncpy(log.date, flightDate, sizeof(log.date)-1);
  } else {
    int hour, minute, second, day, month, year;
    if (getLocalClock(hour, minute, second, day, month, year)) {
      snprintf(log.date, sizeof(log.date), "%02d/%02d/%04d", day, month, year);
    } else {
      strcpy(log.date, "--/--/----");
    }
  }
  log.date[sizeof(log.date)-1] = '\0';

  log.durationSec = automaticFlightAccumMillis / 1000UL;
  log.maxAltitude = flightMaxAltitudeMeters;
  log.maxClimb = flightMaxClimbRate;
  log.fuelUsed = flightFuelUsedL;
  log.maxSpeed = flightMaxGpsSpeedKmh;
  log.distance = flightTotalDistanceKm;
  if (flightEnvironmentValid) {
    log.minTemp = flightMinTempC;
    log.maxTemp = flightMaxTempC;
    log.minHumidity = flightMinHumidity;
    log.maxHumidity = flightMaxHumidity;
  } else {
    log.minTemp = 0.0f; log.maxTemp = 0.0f;
    log.minHumidity = 0.0f; log.maxHumidity = 0.0f;
  }
  log.endTemp = flightEndTempC;
  log.endHumidity = flightEndHumidity;
  log.endEnvironmentValid = flightEndEnvironmentValid;
  log.minG = flightMinG;
  log.maxG = flightMaxG;

  // Il nuovo log va in testa; gli altri scalano di una posizione (il piu'
  // vecchio esce se sono gia' MAX_LOGS). Una sola scrittura NVS.
  loadFlightLogCache();
  const int newCount = min(flightLogCacheCount + 1, MAX_LOGS);
  memmove(&flightLogCache[1], &flightLogCache[0], (newCount - 1) * sizeof(FlightLog));
  flightLogCache[0] = log;
  flightLogCacheCount = newCount;
  saveFlightLogCache();
  if (bleRole == BLE_ROLE_MASTER) {
    bleOutgoingFlightLog = log; bleLogTxIndex = 0; bleLogTxPass = 0; bleLogTxPending = true;
  }
}

void writeTCA9554(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void powerOnAndResetHardware() {
  pinMode(TOUCH_INT_PIN, INPUT_PULLUP);
  Wire.begin(I2C_SDA, I2C_SCL, I2C_BUS_FREQUENCY);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  delay(50);
  writeTCA9554(0x03, 0xF0);
  writeTCA9554(0x01, 0x08);
  delay(50);
  // EXIO1 (bit 0) è il reset del CST816: deve tornare HIGH per abilitarlo.
  // EXIO2/3/4 restano HIGH (reset LCD, CS SD e altro I/O).
  writeTCA9554(0x01, 0x0F);
  delay(250);  // il CST816 ha bisogno di tempo dopo il rilascio del reset
}

void resetI2CBusLocked() {
  AVEA_LOGLN("BME280: reset del bus I2C condiviso");
  Wire.end();

  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delayMicroseconds(10);

  for (int i = 0; i < 9; ++i) {
    pinMode(I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(10);
    pinMode(I2C_SCL, INPUT_PULLUP);
    delayMicroseconds(10);
  }

  pinMode(I2C_SDA, OUTPUT_OPEN_DRAIN);
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(10);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delayMicroseconds(10);
  pinMode(I2C_SDA, INPUT_PULLUP);
  delayMicroseconds(10);

  Wire.begin(I2C_SDA, I2C_SCL, I2C_BUS_FREQUENCY);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  writeTCA9554(0x03, 0xF0);
  // Dopo il recupero I2C rilascia anche EXIO1/reset CST816.
  writeTCA9554(0x01, 0x0F);
  configureCST816NoSleep();  // niente mutex qui dentro: il chiamante lo tiene gia'
}

bool initBME280() {
  if (!sensor_bme.begin(BME280_I2C_ADDR, &Wire)) return false;

  // Ordine dei parametri: modo, temperatura, pressione, umidita', filtro, standby.
  // Pressione x16 (serve per la quota), temperatura x2 e umidita' x1: una
  // conversione dura ~46 ms (con tutto x16 erano ~113 ms, piu' del periodo di
  // lettura di 100 ms, quindi ogni tanto si rileggeva lo stesso campione).
  sensor_bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                          Adafruit_BME280::SAMPLING_X2,
                          Adafruit_BME280::SAMPLING_X16,
                          Adafruit_BME280::SAMPLING_X1,
                          Adafruit_BME280::FILTER_X8,
                          Adafruit_BME280::STANDBY_MS_0_5);
  delay(10);
  return true;
}

void resetVarioAfterBMERecovery() {
  if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    rawAltitudeValid = false;
    sharedAltitudeValid = false;
    sharedVarioValid = false;
    meteoValid = false;
    lastBmeValidMillis = 0;
    varioResetRequested = true;
    xSemaphoreGive(sensorDataMutex);
  }
  // [FIX A] Dopo un recovery la catena riparte da zero: azzera il
  // riferimento dell'ultima lettura grezza accettata.
  lastAcceptedRawAlt = NAN;
}

bool recoverBME280Locked() {
  bmeReady = false;

  for (int retry = 0; retry < 5; ++retry) {
    esp_task_wdt_reset();
    resetI2CBusLocked();
    delay(100UL * (retry + 1));
    esp_task_wdt_reset();

    if (initBME280()) {
      bmeReady = true;
      bmeConsecutiveErrors = 0;
      AVEA_LOGLN("BME280: recuperato e reinizializzato");
      return true;
    }
    esp_task_wdt_reset();
  }

  AVEA_LOGLN("BME280: recovery fallito, nuovo tentativo programmato");
  return false;
}

I2SClass audioI2S;
bool audioCodecReady = false;   // ES8311 configurato e I2S avviato

// ---- ES8311 (codec audio) configurato direttamente con Wire ----
// La libreria es8311 usava il vecchio driver I2C dell'ESP-IDF: insieme a Wire (driver nuovo) il firmware
// si blocca all'avvio ("CONFLICT! driver_ng is not allowed..."), per questo l'audio era disattivato.
// Qui gli stessi registri della libreria si scrivono con Wire, sullo stesso bus degli altri sensori.
static const uint8_t ES8311_I2C_ADDR = 0x18;
static bool es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
static bool es8311Read(uint8_t reg, uint8_t &val) {
  Wire.beginTransmission(ES8311_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)ES8311_I2C_ADDR, (uint8_t)1) != 1) return false;
  val = (uint8_t)Wire.read();
  return true;
}
// Slave I2S, 16 bit, MCLK dal pin = 256 x 24 kHz = 6,144 MHz (stessi valori della tabella della libreria).
static bool es8311Setup(int volumePercent) {
  uint8_t v = 0;
  bool ok = es8311Write(0x00, 0x1F);                 // reset
  delay(20);
  ok = ok && es8311Write(0x00, 0x00);
  ok = ok && es8311Write(0x00, 0x80);                // accensione
  ok = ok && es8311Write(0x01, 0x3F);                // tutti i clock attivi, MCLK dal pin
  ok = ok && es8311Read(0x06, v) && es8311Write(0x06, v & ~0x20);   // SCLK non invertito
  ok = ok && es8311Read(0x02, v) && es8311Write(0x02, v & 0x07);    // pre_div 1, pre_multi x1
  ok = ok && es8311Write(0x03, 0x10);                // single speed, ADC OSR
  ok = ok && es8311Write(0x04, 0x10);                // DAC OSR
  ok = ok && es8311Write(0x05, 0x00);                // divisori ADC/DAC = 1
  ok = ok && es8311Read(0x06, v) && es8311Write(0x06, (v & 0xE0) | 0x03);   // BCLK = MCLK / 4
  ok = ok && es8311Read(0x07, v) && es8311Write(0x07, v & 0xC0);            // LRCK = MCLK / 256
  ok = ok && es8311Write(0x08, 0xFF);
  ok = ok && es8311Read(0x00, v) && es8311Write(0x00, v & 0xBF);            // porta seriale slave
  ok = ok && es8311Write(0x09, 0x0C);                // ingresso DAC: I2S 16 bit
  ok = ok && es8311Write(0x0A, 0x0C);                // uscita ADC: I2S 16 bit
  ok = ok && es8311Write(0x0D, 0x01);                // parte analogica accesa
  ok = ok && es8311Write(0x0E, 0x02);
  ok = ok && es8311Write(0x12, 0x00);                // DAC acceso
  ok = ok && es8311Write(0x13, 0x10);                // uscita verso l'amplificatore
  ok = ok && es8311Write(0x1C, 0x6A);
  ok = ok && es8311Write(0x37, 0x08);                // equalizzatore DAC escluso
  ok = ok && es8311Write(0x17, 0xC8);                // microfono: come la libreria (non usato)
  ok = ok && es8311Write(0x14, 0x1A);
  const int vol = constrain(volumePercent, 0, 100);
  ok = ok && es8311Write(0x32, (uint8_t)(vol == 0 ? 0 : vol * 256 / 100 - 1));   // volume
  return ok;
}

// Amplificatore (NS4150B) acceso solo con AUDIO ON: spento non consuma e non fruscia.
void audioApplyEnabled() {
  if (audioCodecReady) digitalWrite(PA_CTRL, isAudioEnabled ? HIGH : LOW);
}

void initAudioI2S() {
  pinMode(PA_CTRL, OUTPUT);
  digitalWrite(PA_CTRL, LOW);
  // Prima l'I2S: il codec ha bisogno del clock MCLK gia' presente quando viene configurato.
  audioI2S.setPins(I2S_BCLK, I2S_LRCK, I2S_DOUT, I2S_DIN, I2S_MCLK);
  if (!audioI2S.begin(I2S_MODE_STD, AUDIO_SAMPLE_RATE, I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    AVEA_LOGLN("Audio: inizializzazione I2S fallita");
    return;
  }
  const bool locked = (i2cMutex != nullptr && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(200)) == pdTRUE);
  const bool ok = es8311Setup(80);
  if (locked) xSemaphoreGive(i2cMutex);
  if (!ok) {
    AVEA_LOGLN("Audio: ES8311 non trovato o non configurato");
    audioI2S.end();
    return;
  }
  audioCodecReady = true;
  audioApplyEnabled();
  AVEA_LOGLN("Audio: ES8311 + I2S + NS4150B pronti");
}

// ---- Audio in un compito separato (secondo processore): playTone() e silenceAudio() mettono la richiesta
// in coda e tornano subito, cosi' i bip del variometro (fino a 300 ms) non fermano piu' display e tocco.
// Le richieste vengono suonate in ordine; se la coda e' piena la nuova richiesta si salta.
struct AudioReq { int16_t freq; int16_t dur; };
static QueueHandle_t audioQueue = nullptr;
static void audioRenderTone(int freqHz, int durationMs) {
  const int samples = (AUDIO_SAMPLE_RATE * durationMs) / 1000;
  const float step = 2.0f * PI * freqHz / AUDIO_SAMPLE_RATE;
  const float amplitude = 12000.0f * audioVolumePercent / 100.0f;
  static float phase = 0.0f;
  int16_t buffer[128];
  for (int offset = 0; offset < samples; offset += 128) {
    int count = min(128, samples - offset);
    for (int i = 0; i < count; ++i) {
      buffer[i] = (int16_t)(sinf(phase) * amplitude);
      phase += step;
      if (phase >= 2.0f * PI) phase -= 2.0f * PI;
    }
    audioI2S.write((uint8_t*)buffer, count * sizeof(int16_t));
  }
}
static void audioRenderSilence() {
  static int16_t silence[128] = {0};
  audioI2S.write((uint8_t*)silence, sizeof(silence));
}
static void audioTask(void *) {
  AudioReq r;
  for (;;) {
    if (xQueueReceive(audioQueue, &r, portMAX_DELAY) != pdTRUE) continue;
    if (!audioCodecReady) continue;
    if (r.freq > 0) audioRenderTone(r.freq, r.dur);
    else audioRenderSilence();
  }
}
static bool audioEnsureTask() {
  if (audioQueue != nullptr) return true;
  audioQueue = xQueueCreate(6, sizeof(AudioReq));
  if (audioQueue == nullptr) return false;
  if (xTaskCreatePinnedToCore(audioTask, "AudioTask", 3072, NULL, 1, NULL, 0) != pdPASS) {
    vQueueDelete(audioQueue); audioQueue = nullptr; return false;
  }
  return true;
}

void playTone(int freqHz, int durationMs) {
  if (!isAudioEnabled || !audioCodecReady || freqHz <= 0 || durationMs <= 0) return;
  if (!audioEnsureTask()) { audioRenderTone(freqHz, durationMs); return; }   // senza compito: come prima
  AudioReq r; r.freq = (int16_t)min(freqHz, 20000); r.dur = (int16_t)min(durationMs, 5000);
  xQueueSend(audioQueue, &r, 0);
}

void silenceAudio() {
  if (!audioCodecReady) return;
  if (!audioEnsureTask()) { audioRenderSilence(); return; }
  AudioReq r; r.freq = 0; r.dur = 0;
  xQueueSend(audioQueue, &r, 0);
}

// ========== FONT VLW CARICATI UNA VOLTA SOLA ==========
// Prima ogni scritta caricava il font VLW (lettura della tabella di tutti i caratteri e 5 blocchi di
// memoria allocati) e lo scaricava subito dopo. Ora ogni font viene letto una sola volta, alla prima
// scritta, e poi solo "agganciato" al display o allo sprite: stesso aspetto, molto meno lavoro.
// vlwSet() sostituisce loadFont(), vlwRelease() sostituisce unloadFont() (torna al font di base,
// come prima). Tutti i font VLW passano da qui: non usare loadFont()/unloadFont() direttamente.
static const int VLW_SLOTS = 8;
struct VlwSlot {
  const uint8_t *src = nullptr;
  lgfx::PointerWrapper data;
  lgfx::VLWfont font;
  bool tried = false;
  bool ok = false;
};
static VlwSlot vlwSlots[VLW_SLOTS];

const lgfx::IFont *vlwFontFor(const uint8_t *src) {
  for (int i = 0; i < VLW_SLOTS; ++i) {
    VlwSlot &sl = vlwSlots[i];
    if (sl.tried && sl.src == src) return sl.ok ? &sl.font : nullptr;
    if (!sl.tried) {
      sl.tried = true;
      sl.src = src;
      sl.data.set(src);
      sl.ok = sl.font.loadFont(&sl.data);
      if (!sl.ok) sl.font.unloadFont();
      return sl.ok ? &sl.font : nullptr;
    }
  }
  return nullptr;   // piu' font diversi degli slot: si usa il font di riserva del chiamante
}

bool vlwSet(lgfx::LGFXBase &g, const uint8_t *src) {
  const lgfx::IFont *f = vlwFontFor(src);
  if (f == nullptr) return false;
  g.setFont(f);
  return true;
}

void vlwRelease(lgfx::LGFXBase &g) {
  g.setFont(&lgfx::fonts::Font0);
}

int drawNumericVlw(const char *text, int x, int y, decltype(MC_DATUM) datum, uint16_t fg, uint16_t bg, float scale, int clearMaxChars) {
  bool loaded = vlwSet(lcd, VarioNumbers80);
  if (loaded) {
    lcd.setTextSize(scale, scale);
  } else {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
    lcd.setTextSize(scale * 0.8f, scale * 0.8f);
  }
  lcd.setTextDatum(datum);
  uint16_t actualFg = displayTheme == THEME_COLOR ? fg : TFT_BLACK;
  uint16_t actualBg = displayTheme == THEME_COLOR ? bg : themeBg();
  lcd.setTextColor(actualFg, actualBg);

  // [FIX GHOST DIGIT] drawString riempie lo sfondo solo sulla larghezza del
  // testo NUOVO: se il valore precedente era piu' largo (es. "100" -> "9"),
  // i pixel ai lati restano quelli vecchi. Se il chiamante passa
  // clearMaxChars > 0, puliamo prima un'area larga quanto il caso peggiore
  // (tutte cifre '0' piu' un segno meno), centrata su (x,y) come il testo.
  if (clearMaxChars > 0) {
    char widest[12];
    int n = clearMaxChars < (int)sizeof(widest) - 1 ? clearMaxChars : (int)sizeof(widest) - 1;
    for (int i = 0; i < n; i++) widest[i] = '0';
    widest[0] = '-';
    widest[n] = '\0';
    int maxW = lcd.textWidth(widest);
    int fontH = lcd.fontHeight();
    int halfW = maxW / 2 + 4;
    int halfH = fontH / 2 + 4;
    lcd.fillRect(x - halfW, y - halfH, halfW * 2, halfH * 2, actualBg);
  }

  int width = lcd.textWidth(text);
  lcd.drawString(text, x, y);
  if (loaded) vlwRelease(lcd);
  return width;
}

// Superficie su cui scrive drawUiVlw: di serie il display; le pagine ridisegnate per intero (avvisi CARB ICE,
// VISIBILITY, DENSITY ALTITUDE) la puntano allo sprite e lo spingono in un colpo solo, senza lampeggio.
lgfx::LGFXBase *uiTarget = nullptr;
static inline lgfx::LGFXBase &UT() { return uiTarget ? *uiTarget : (lgfx::LGFXBase &)lcd; }
int drawUiVlw(const char *text, int x, int y, decltype(MC_DATUM) datum, uint16_t fg, uint16_t bg, float scale) {
  bool loaded = vlwSet(UT(), UiFont32);
  if (loaded) {
    UT().setTextSize(scale, scale);
  } else {
    UT().setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    UT().setTextSize(scale * 1.5f, scale * 1.5f);
  }
  UT().setTextDatum(datum);
  uint16_t actualFg = displayTheme == THEME_COLOR ? fg : TFT_BLACK;
  uint16_t actualBg = displayTheme == THEME_COLOR ? bg : themeBg();
  UT().setTextColor(actualFg, actualBg);
  int width = UT().textWidth(text);
  UT().drawString(text, x, y);
  if (loaded) vlwRelease(UT());
  return width;
}

int drawNumericVlwSprite(LGFX_Sprite &sprite, const char *text, int x, int y,
                         decltype(MC_DATUM) datum, uint16_t fg, uint16_t bg, float scale) {
  bool loaded = vlwSet(sprite, VarioNumbers80);
  if (loaded) {
    sprite.setTextSize(scale, scale);
  } else {
    sprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
    sprite.setTextSize(scale * 0.8f, scale * 0.8f);
  }
  sprite.setTextDatum(datum);
  const uint16_t actualFg = displayTheme == THEME_COLOR ? fg : TFT_BLACK;
  const uint16_t actualBg = displayTheme == THEME_COLOR ? bg : themeBg();
  sprite.setTextColor(actualFg, actualBg);
  const int width = sprite.textWidth(text);
  sprite.drawString(text, x, y);
  if (loaded) vlwRelease(sprite);
  return width;
}

int drawUiVlwSprite(LGFX_Sprite &sprite, const char *text, int x, int y,
                    decltype(MC_DATUM) datum, uint16_t fg, uint16_t bg, float scale) {
  bool loaded = vlwSet(sprite, UiFont32);
  if (loaded) {
    sprite.setTextSize(scale, scale);
  } else {
    sprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    sprite.setTextSize(scale * 1.5f, scale * 1.5f);
  }
  sprite.setTextDatum(datum);
  const uint16_t actualFg = displayTheme == THEME_COLOR ? fg : TFT_BLACK;
  const uint16_t actualBg = displayTheme == THEME_COLOR ? bg : themeBg();
  sprite.setTextColor(actualFg, actualBg);
  const int width = sprite.textWidth(text);
  sprite.drawString(text, x, y);
  if (loaded) vlwRelease(sprite);
  return width;
}

// Larghezza di un testo nel carattere UiFont32 (senza disegnarlo)
int uiVlwWidthSprite(LGFX_Sprite &sprite, const char *text, float scale) {
  bool loaded = vlwSet(sprite, UiFont32);
  if (loaded) sprite.setTextSize(scale, scale);
  else { sprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); sprite.setTextSize(scale * 1.5f, scale * 1.5f); }
  const int width = sprite.textWidth(text);
  if (loaded) vlwRelease(sprite);
  return width;
}

void handleVarioAudio() {
  if (!isAudioEnabled || !audioCodecReady) {
    if (varioAudioActive) { silenceAudio(); varioAudioActive = false; lastVarioAudioFreq = 0; }
    return;
  }
  // In demo i toni del variometro suonano anche loro: playTone() ora non blocca piu' il loop (compito audio separato).
  float audioVario = 0.0f;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)2) == pdTRUE) {
    audioVario = sharedVarioKalman;
    xSemaphoreGive(sensorDataMutex);
  }
  float climbThreshold = varioUpThreshold;
  float sinkThreshold = varioDownThreshold;
  if (waveMode) {
    climbThreshold += WAVE_CLIMB_OFFSET;
    sinkThreshold = WAVE_SINK_THRESHOLD;
  }
  float idleDeadband = 0.18f * climbThreshold;
  if (idleDeadband < 0.08f) idleDeadband = 0.08f;
  unsigned long now = millis();
  if (fabs(audioVario) < idleDeadband) {
    if (varioAudioActive) silenceAudio();
    varioAudioActive = false;
    lastVarioAudioFreq = 0;
    return;
  }
  // [FIX D] Il pavimento di buzzThreshold era un valore fisso (0.12) scollegato
  // da idleDeadband: con soglie di salita basse (es. 0.2 m/s) idleDeadband
  // scendeva a 0.08 lasciando una fascia 0.08-0.12 m/s completamente muta
  // (ne' deadband ne' buzz). Ora il pavimento e' idleDeadband stesso, cosi'
  // il buzz riparte esattamente dove finisce il silenzio, senza buchi.
  float buzzThreshold = climbThreshold * 0.38f;
  if (buzzThreshold < idleDeadband) buzzThreshold = idleDeadband;
  if (!buzzEnabled) buzzThreshold = climbThreshold;

  if (audioVario < VARIO_EMERGENCE_THRESHOLD) {
    if (now - lastVarioBeepMillis > 500) {
      lastVarioBeepMillis = now;
      playTone(2000, 150);
      lastVarioAudioFreq = 2000;
      varioAudioActive = true;
    } else if (now - lastVarioBeepMillis > 250 && lastVarioAudioFreq != 1000) {
      playTone(1000, 150);
      lastVarioAudioFreq = 1000;
      varioAudioActive = true;
    }
  } else if (audioVario > climbThreshold && audioVario < 15.0f) {
    float beepPeriod = 350.0f - audioVario * 10.0f;
    if (beepPeriod < 50.0f) beepPeriod = 50.0f;
    if (now - lastVarioBeepMillis > (unsigned long)beepPeriod) {
      lastVarioBeepMillis = now;
      int freq = 700 + (int)(60.0f * audioVario);
      if (freq > 1600) freq = 1600;
      int duration = 300 - (int)(audioVario * 5.0f);
      if (duration < 50) duration = 50;
      playTone(freq, duration);
      lastVarioAudioFreq = freq;
      varioAudioActive = true;
    }
  } else if (audioVario >= buzzThreshold && audioVario <= climbThreshold) {
    if (now - lastVarioBuzzMillis > 150) {
      lastVarioBuzzMillis = now;
      playTone(650, 22);
      lastVarioAudioFreq = 650;
      varioAudioActive = true;
    }
  } else if (audioVario < sinkThreshold) {
    int targetFreq = 700 + (int)(130.0f * (fabs(audioVario) - 0.2f));
    if (targetFreq < 700) targetFreq = 700;
    if (targetFreq > 2200) targetFreq = 2200;

    if (abs(targetFreq - sinkToneActiveFreq) >= 40) {
      sinkToneActiveFreq = targetFreq;
    }

    if (now - lastVarioBeepMillis > 200) {
      lastVarioBeepMillis = now;
      playTone(sinkToneActiveFreq, 100);
      lastVarioAudioFreq = sinkToneActiveFreq;
      varioAudioActive = true;
    }
  } else {
    sinkToneActiveFreq = 700;
    if (varioAudioActive) silenceAudio();
    varioAudioActive = false;
    lastVarioAudioFreq = 0;
  }
}

static bool touchAddressResponds(TwoWire &bus) {
  bus.beginTransmission(TOUCH_ADDR);
  return bus.endTransmission() == 0;
}

// Il CST816 non risponde all'indirizzo se e' in sleep o rimasto in reset: si prova a
// risvegliarlo con un impulso su EXIO1 (solo bit0 del TCA9554: il reset LCD non si tocca).
static bool touchProbeWithReset(int attempts) {
  if (touchAddressResponds(Wire)) return true;
  for (int i = 0; i < attempts; ++i) {
    writeTCA9554(0x01, 0x0E);   // EXIO1 basso = reset CST816
    delay(20);
    writeTCA9554(0x01, 0x0F);   // rilascio
    delay(250);                  // il CST816 ha bisogno di tempo dopo il rilascio del reset
    if (touchAddressResponds(Wire)) {
      AVEA_LOGF("[TOUCH] CST816 risponde dopo il reset (tentativo %d)\n", i + 1);
      return true;
    }
  }
  return false;
}

static bool touchIntAttached = false;

static void initTouchI2cBus() {
  touchI2cBus = &Wire;
  touchI2cReady = false;

  // La revisione 1.85C usa il bus condiviso GPIO11/10; altre revisioni
  // documentano il CST816 su GPIO1/3. Seleziona il bus che risponde a 0x15.
  if (touchProbeWithReset(3)) {
    touchI2cReady = true;
    AVEA_LOGLN("Touch CST816 rilevato su I2C GPIO11/10");
    return;
  }

  if (!Wire1.begin(TOUCH_ALT_SDA_PIN, TOUCH_ALT_SCL_PIN, I2C_BUS_FREQUENCY)) {
    AVEA_LOGLN("Touch CST816: avvio bus alternativo GPIO1/3 fallito");
    return;
  }
  Wire1.setTimeOut(I2C_TIMEOUT_MS);
  if (touchAddressResponds(Wire1)) {
    touchI2cBus = &Wire1;
    touchI2cReady = true;
    AVEA_LOGLN("Touch CST816 rilevato su I2C GPIO1/3");
  } else {
    AVEA_LOGLN("Touch CST816 non trovato su GPIO11/10 ne' GPIO1/3");
  }
}

// Disabilita l'auto-sleep del CST816 e abilita INT su tocco/cambio.
// Scrive solo 0xFA (IrqCtl) e 0xFE (DisAutoSleep): 0xFB/0xFC/0xFD NON vanno toccati.
void configureCST816NoSleep() {
  if (!touchI2cReady || touchI2cBus == nullptr) return;
  TwoWire *bus = touchI2cBus;
  bus->beginTransmission(TOUCH_ADDR); bus->write(0xFA); bus->write(0x60);
  const bool okA = (bus->endTransmission() == 0);
  bus->beginTransmission(TOUCH_ADDR); bus->write(0xFE); bus->write(0x01);
  const bool okE = (bus->endTransmission() == 0);
  (void)okA; (void)okE;
}

static void touchRetryDetect() {
  static unsigned long nextTryMs = 3000UL;
  static int tries = 0;
  const unsigned long now = millis();
  if ((long)(now - nextTryMs) < 0) return;
  ++tries;
  nextTryMs = now + (tries < 10 ? 3000UL : 15000UL);
  if (i2cMutex == nullptr || xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) != pdTRUE) return;
  const bool ok = touchProbeWithReset(1);
  if (ok) {
    touchI2cBus = &Wire;
    touchI2cReady = true;
    configureCST816NoSleep();
    touchIntFlag = true;
    if (!touchIntAttached) {
      attachInterrupt(digitalPinToInterrupt(TOUCH_INT_PIN), onTouchInt, FALLING);
      touchIntAttached = true;
    }
    AVEA_LOGLN("[TOUCH] CST816 rilevato dopo l'avvio");
  }
  xSemaphoreGive(i2cMutex);
}

bool getTouch(uint16_t *x, uint16_t *y, bool *sampleValid = nullptr) {
  if (sampleValid != nullptr) *sampleValid = false;
  touchSkippedNoInt = false;
  if (!touchI2cReady || touchI2cBus == nullptr) { touchRetryDetect(); return false; }
  static unsigned long lastReadMs = 0;
  static bool lastDown = false;
  const unsigned long nowMs = millis();
  const bool intPending = touchIntFlag;
  // Se dopo 5 s dall'avvio non e' mai arrivato un INT (linea INT non collegata o non attiva su
  // questa scheda), il touch si legge a polling ogni 25 ms: senza questo non si riceverebbe mai un tocco.
  const bool intDead = (touchIntCount == 0 && nowMs > 5000UL);
  static bool intDeadReported = false;
  if (intDead && !intDeadReported) {
    intDeadReported = true;
    AVEA_LOGLN("[TOUCH] nessun INT ricevuto dall'avvio: lettura a polling");
  }
  const bool fallbackDue = (lastDown && (nowMs - lastReadMs > TOUCH_POLL_FALLBACK_MS)) ||
                           (intDead && (nowMs - lastReadMs >= 25UL));
  // Il CST816 risponde bene solo dopo un INT: senza INT non leggere (evita dati sporchi).
  if (!intPending && !fallbackDue) { touchSkippedNoInt = true; return false; }
  TwoWire *bus = touchI2cBus;
  const bool lockSharedBus = (bus == &Wire && i2cMutex != nullptr);
  if (lockSharedBus && xSemaphoreTake(i2cMutex, (TickType_t)10) != pdTRUE) return false;
  touchIntFlag = false;   // azzerato solo dopo aver preso il bus: un INT nuovo durante la lettura resta in coda
  lastReadMs = nowMs;

  // Lettura con repeated START (indirizzo registro + 6 byte sotto lo stesso mutex).
  // Un errore o una lettura corta NON equivale a un rilascio del dito.
  bus->beginTransmission(TOUCH_ADDR);
  bus->write(0x01);
  if (bus->endTransmission(false) != 0) {  // repeated START
    if (lockSharedBus) xSemaphoreGive(i2cMutex);
    return false;
  }
  const size_t received = bus->requestFrom((uint8_t)TOUCH_ADDR, (size_t)6);
  if (received < 6 || bus->available() < 6) {
    while (bus->available()) bus->read();
    if (lockSharedBus) xSemaphoreGive(i2cMutex);
    return false;
  }

  const uint8_t gestureId = (uint8_t)bus->read();
  const uint8_t touchNumRaw = (uint8_t)bus->read();
  const uint8_t touchNum = touchNumRaw & 0x0F;
  const uint8_t xHi = (uint8_t)bus->read();
  const uint8_t xLo = (uint8_t)bus->read();
  const uint8_t yHi = (uint8_t)bus->read();
  const uint8_t yLo = (uint8_t)bus->read();
  if (lockSharedBus) xSemaphoreGive(i2cMutex);

  if (touchNum == 0) {
    lastDown = false;
    if (sampleValid != nullptr) *sampleValid = true;
    return false;
  }
  // Il CST816 è a singolo punto: conteggi oltre 1 sono dati anomali, non tocchi.
  if (touchNum != 1) {
    return false;
  }
  const uint16_t candidateX = ((xHi & 0x0F) << 8) | xLo;
  const uint16_t candidateY = ((yHi & 0x0F) << 8) | yLo;
  if (candidateX >= TOUCH_MAX_X || candidateY >= TOUCH_MAX_Y) return false;
  // Il pannello ha segnalato falsi DOWN a (0,0) senza gesto: non usarli per navigare.
  if (candidateX == 0 && candidateY == 0 && gestureId == 0) {
    return false;
  }

  lastDown = true;
  // ROTATE DISPLAY 180: il pannello touch non ruota con lcd.setRotation(), le coordinate si girano qui
  *x = displayRotated180 ? (uint16_t)(TOUCH_MAX_X - 1 - candidateX) : candidateX;
  *y = displayRotated180 ? (uint16_t)(TOUCH_MAX_Y - 1 - candidateY) : candidateY;
  if (sampleValid != nullptr) *sampleValid = true;
  return true;
}

// [FIX 1] calculateVario() consuma newAltitudeSample: la regressione lineare
// e il filtro di Kalman avanzano solo su una lettura BME280 reale (10 Hz).
void calculateVario() {
  unsigned long currentMillis = millis();
  const double tempo = (double)currentMillis;
  float measuredAltitude;
  bool haveAltitude = false;
  bool resetRequested = false;
  bool hasNewSample = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    measuredAltitude = rawAltitudeShared;
    haveAltitude = rawAltitudeValid;
    resetRequested = varioResetRequested;
    varioResetRequested = false;
    hasNewSample = newAltitudeSample;
    newAltitudeSample = false;
    xSemaphoreGive(sensorDataMutex);
  } else {
    return;
  }
  if (resetRequested) {
    varioBufferInitialized = false;
    kalman_P_alt = 1.0f;
    kalman_P_vario = 1.0f;
    kalman_vario = 0.0f;
    vario = 0.0f;
    vario_kalman = 0.0f;
    if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
      sharedVarioKalman = 0.0f;
      xSemaphoreGive(sensorDataMutex);
    }
  }
  if (!haveAltitude) return;
  if (!hasNewSample) return;

  if (!varioBufferInitialized) {
    for (int i = 0; i <= vario_maxsamples; i++) {
      alt[i] = measuredAltitude;
      // [FIX 1] Spaziatura coerente col periodo reale del BME280 (100 ms).
      tim[i] = tempo - ((vario_maxsamples - i) * 100);
    }
    kalman_alt = measuredAltitude;
    varioBufferInitialized = true;
  }

  kalman_P_alt += kalman_Q_alt;
  float K_alt = kalman_P_alt / (kalman_P_alt + kalman_R_alt);
  kalman_alt = kalman_alt + K_alt * (measuredAltitude - kalman_alt);
  kalman_P_alt = (1 - K_alt) * kalman_P_alt;

  memmove(&alt[0], &alt[1], vario_maxsamples * sizeof(float));
  memmove(&tim[0], &tim[1], vario_maxsamples * sizeof(double));
  alt[vario_maxsamples] = kalman_alt;
  tim[vario_maxsamples] = tempo;

  float N1 = 0, N2 = 0, N3 = 0, D1 = 0, D2 = 0;
  int num_samples_to_use = vario_samples;
  const double stime = tim[vario_maxsamples - vario_samples + 1];
  for (int cc = (vario_maxsamples - vario_samples + 1); cc <= vario_maxsamples; cc++) {
    const float dt = (float)(tim[cc] - stime);  // differenza piccola: float basta
    N1 += dt * alt[cc];
    N2 += dt;
    N3 += alt[cc];
    D1 += dt * dt;
    D2 += dt;
  }
  float denominator = (num_samples_to_use * D1 - D2 * D2);
  if (denominator != 0) vario = 1000 * ((num_samples_to_use * N1) - N2 * N3) / denominator;
  else vario = 0;

  kalman_P_vario += kalman_Q_vario;
  float K_vario = kalman_P_vario / (kalman_P_vario + kalman_R_vario);
  kalman_vario = kalman_vario + K_vario * (vario - kalman_vario);
  kalman_P_vario = (1 - K_vario) * kalman_P_vario;
  vario_kalman = kalman_vario;

  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    sharedFilteredAlt = kalman_alt;
    sharedVarioKalman = vario_kalman;
    sharedAltitudeValid = true;
    sharedVarioValid = true;
    if (vario_kalman > maxClimbRate) maxClimbRate = vario_kalman;
    if (vario_kalman < maxSinkRate) maxSinkRate = vario_kalman;
    xSemaphoreGive(sensorDataMutex);
  }
}

void gpsSendPairCommand(const char *body) {
  uint8_t checksum = 0;
  for (const char *p = body; *p != '\0'; ++p) checksum ^= (uint8_t)*p;

  char command[64];
  snprintf(command, sizeof(command), "$%s*%02X\r\n", body, checksum);
  GpsSerial.print(command);
  GpsSerial.flush();
  delay(80);
}

void gpsSetRate10Hz() {
  gpsSendPairCommand("PAIR050,100");      // 5 Hz cambiando PAIR050,100 in PAIR050,200
}

void gpsConfigureNmeaMinimal() {
  gpsSendPairCommand("PAIR066,1,1,1,1,1,0");  // costellazioni: GPS,GLONASS,Galileo,BDS,QZSS,(NavIC off)
  // [FIX GPS] A 10 Hz con 30+ satelliti il flusso NMEA (GSV/GSA/GLL/VTG ad ogni fix)
  // supera i 115200 baud: il modulo scarta frasi e lo strumento "perde" il GPS.
  // PAIR062,<tipo>,<ogni N fix>: 0=GGA 1=GLL 2=GSA 3=GSV 4=RMC 5=VTG (N=0 disattiva)
  gpsSendPairCommand("PAIR062,1,0");   // GLL off (non usata)
  gpsSendPairCommand("PAIR062,5,0");   // VTG off (velocita' presa da RMC)
  gpsSendPairCommand("PAIR062,3,10");  // GSV a 1 Hz (basta per satelliti/costellazioni)
  gpsSendPairCommand("PAIR062,2,10");  // GSA a 1 Hz (tipo di fix)
}

// [FIX GPS] Watchdog del flusso NMEA: se per 3 s non arriva nessuna frase valida
// (dopo che ne erano arrivate) riapre la seriale e rimanda la configurazione.
static unsigned long gpsLastGoodSentenceMs = 0;
static unsigned long gpsLastRecoverMs = 0;

static void gpsWatchdogStep() {
  const unsigned long now = millis();
  if (bleRole == BLE_ROLE_SLAVE || gpsLastGoodSentenceMs == 0) return;  // lo slave non ha GPS
  if (now - gpsLastGoodSentenceMs > 3000UL && now - gpsLastRecoverMs > 5000UL) {
    gpsLastRecoverMs = now;
    GpsSerial.end();
    GpsSerial.setRxBufferSize(2048);
    GpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    nmeaBufferLen = 0;
    gpsConfigureNmeaMinimal();
    gpsSetRate10Hz();
  }
}

// [FIX RIAVVII] Helper senza allocazioni per leggere i campi separati da
// virgola di una frase NMEA. Restituisce il puntatore all'inizio del campo
// N (0-based, il campo 0 e' la parte prima della prima virgola, es.
// "$GPRMC"), oppure nullptr se la frase non ha abbastanza campi.
static const char* nmeaField(const char *sentence, int fieldIndex) {
  const char *p = sentence;
  for (int idx = 0; idx < fieldIndex; idx++) {
    p = strchr(p, ',');
    if (!p) return nullptr;
    p++;
  }
  return p;
}

// Lunghezza del campo che inizia in p, fino alla prossima virgola, al '*'
// del checksum o alla fine stringa.
static size_t nmeaFieldLen(const char *p) {
  if (!p) return 0;
  size_t len = 0;
  while (p[len] != '\0' && p[len] != ',' && p[len] != '*') len++;
  return len;
}

void parseNmeaSentence(const char *sentence) {
  if (strncmp(sentence, "$GNRMC", 6) == 0 || strncmp(sentence, "$GPRMC", 6) == 0) {
    const char *timeField  = nmeaField(sentence, 1);
    const char *statusField = nmeaField(sentence, 2);
    const bool statusOk = statusField && *statusField == 'A';

    lastGpsMessageMillis = millis();
    lastRmcMillis = millis();
    if (statusOk) {
      if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
        gpsRmcStatusValid = true;
        gpsDataValid = (gpsRmcStatusValid && gps_fix_quality > 0 && lastGgaMillis != 0 && millis() - lastGgaMillis < 1500UL);
        if (gpsDataValid) lastOwnFixMillis = millis();
        if (nmeaFieldLen(timeField) >= 6) {
          gps_utc_hours   = (timeField[0] - '0') * 10 + (timeField[1] - '0');
          gps_utc_minutes = (timeField[2] - '0') * 10 + (timeField[3] - '0');
          gps_utc_seconds = (timeField[4] - '0') * 10 + (timeField[5] - '0');
        }
        const char *latField    = nmeaField(sentence, 3);
        const char *latDirField = nmeaField(sentence, 4);
        const char *lonField    = nmeaField(sentence, 5);
        const char *lonDirField = nmeaField(sentence, 6);

        if (nmeaFieldLen(latField) > 0 && nmeaFieldLen(lonField) > 0) {
          float lat = atof(latField);
          float lon = atof(lonField);
          float lat_deg = int(lat / 100);
          float lat_min = lat - (lat_deg * 100);
          gps_latitude = lat_deg + (lat_min / 60.0f);
          if (latDirField && *latDirField == 'S') gps_latitude = -gps_latitude;
          float lon_deg = int(lon / 100);
          float lon_min = lon - (lon_deg * 100);
          gps_longitude = lon_deg + (lon_min / 60.0f);
          if (lonDirField && *lonDirField == 'W') gps_longitude = -gps_longitude;
        }

        const char *speedField  = nmeaField(sentence, 7);
        const char *courseField = nmeaField(sentence, 8);
        const char *dateField   = nmeaField(sentence, 9);

        if (nmeaFieldLen(speedField) > 0) {
          gps_ground_speed_kmh = atof(speedField) * 1.852f;
          if (gps_ground_speed_kmh < GS_ZERO_THRESHOLD_KMH) gps_ground_speed_kmh = 0.0f;
          if (gps_ground_speed_kmh > maxGpsSpeedKmh) maxGpsSpeedKmh = gps_ground_speed_kmh;
        }
        if (nmeaFieldLen(courseField) > 0) gps_course = atof(courseField);

        if (nmeaFieldLen(dateField) == 6) {
          gps_utc_day   = (dateField[0] - '0') * 10 + (dateField[1] - '0');
          gps_utc_month = (dateField[2] - '0') * 10 + (dateField[3] - '0');
          gps_utc_year  = 2000 + (dateField[4] - '0') * 10 + (dateField[5] - '0');
        }
        xSemaphoreGive(sensorDataMutex);
        if (nmeaFieldLen(timeField) >= 6 && nmeaFieldLen(dateField) == 6) {
          syncSoftwareClockFromGps(gps_utc_hours, gps_utc_minutes, gps_utc_seconds,
                                   gps_utc_day, gps_utc_month, gps_utc_year);
        }
      }
    } else if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
      gpsRmcStatusValid = false;
      gpsDataValid = false;
      gps_fix_quality = 0;
      gps_ground_speed_kmh = 0.0f;
      xSemaphoreGive(sensorDataMutex);
    }
  } else if (strncmp(sentence, "$GNGGA", 6) == 0 || strncmp(sentence, "$GPGGA", 6) == 0) {
    const char *fixQualityField = nmeaField(sentence, 6);
    const char *satellitesField = nmeaField(sentence, 7);
    const char *altField        = nmeaField(sentence, 9);
    lastGpsMessageMillis = millis();
    lastGgaMillis = millis();
    if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
      if (nmeaFieldLen(fixQualityField) > 0) {
        gps_fix_quality = atoi(fixQualityField);
        gpsDataValid = (gpsRmcStatusValid && gps_fix_quality > 0 && lastRmcMillis != 0 && millis() - lastRmcMillis < 1500UL);
        if (gpsDataValid) lastOwnFixMillis = millis();
      }
      if (nmeaFieldLen(satellitesField) > 0) gps_satellites = atoi(satellitesField);
      if (nmeaFieldLen(altField) > 0) gps_altitude = atof(altField);
      xSemaphoreGive(sensorDataMutex);
    }
  } else if (strlen(sentence) > 6 && strncmp(sentence + 3, "GSA", 3) == 0) {
    const char *fixTypeField = nmeaField(sentence, 2);
    if (nmeaFieldLen(fixTypeField) > 0) {
      int ft = atoi(fixTypeField);
      if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
        gps_fix_type = ft;
        xSemaphoreGive(sensorDataMutex);
      }
    }
  } else if (strlen(sentence) > 6 && strncmp(sentence + 3, "GSV", 3) == 0) {
    // Il 3o campo della GSV e' il numero di satelliti in vista per QUEL sistema.
    // Molti ricevitori multi-GNSS emettono la frase ad intervalli fissi anche
    // con 0 satelliti in vista, quindi la sola presenza della frase non basta:
    // va controllato il conteggio per sapere se la costellazione e' davvero visibile.
    const char *satsInViewField = nmeaField(sentence, 3);
    int satsInView = (nmeaFieldLen(satsInViewField) > 0) ? atoi(satsInViewField) : 0;

    if (satsInView > 0) {
      if (strncmp(sentence, "$GPGSV", 6) == 0 || strncmp(sentence, "$GNGSV", 6) == 0) {
        gps_has_gps = true; gps_last_gsv_gps = millis();
      } else if (strncmp(sentence, "$GLGSV", 6) == 0) {
        gps_has_glonass = true; gps_last_gsv_glonass = millis();
      } else if (strncmp(sentence, "$GAGSV", 6) == 0) {
        gps_has_galileo = true; gps_last_gsv_galileo = millis();
      } else if (strncmp(sentence, "$GBGSV", 6) == 0 || strncmp(sentence, "$BDGSV", 6) == 0) {
        gps_has_bds = true; gps_last_gsv_bds = millis();
      } else if (strncmp(sentence, "$GQGSV", 6) == 0) {
        gps_has_qzss = true; gps_last_gsv_qzss = millis();
      }
    }
  }
}

// [FIX RIAVVII] Checksum calcolato direttamente sul char* del buffer fisso,
// senza costruire String temporanee.
bool nmeaHasValidChecksum(const char *s, size_t len) {
  if (len < 9 || s[0] != '$') return false;
  int star = -1;
  for (size_t i = 1; i < len; ++i) {
    if (s[i] == '*') { star = (int)i; break; }
  }
  if (star < 0 || (size_t)(star + 2) >= len) return false;
  uint8_t cs = 0;
  for (int i = 1; i < star; ++i) cs ^= (uint8_t)s[i];
  char expected[3];
  sprintf(expected, "%02X", cs);
  char c1 = (char)toupper((unsigned char)s[star + 1]);
  char c2 = (char)toupper((unsigned char)s[star + 2]);
  return c1 == expected[0] && c2 == expected[1];
}

// [DEMO] Rampe lineari sincronizzate su un ciclo di due minuti.
// La prima metà sale da minimo a massimo in 60 s; la seconda torna al minimo.
const float DEMO_HALF_CYCLE_SECONDS = 60.0f;
const float DEMO_CYCLE_SECONDS = 2.0f * DEMO_HALF_CYCLE_SECONDS;
const float DEMO_SPEED_MIN_KMH = 5.0f;
const float DEMO_SPEED_MAX_KMH = 220.0f;   // oltre 200 km/h: si prova il passaggio della scala a 0-300
const float DEMO_VARIO_MIN_MS = -8.0f;
const float DEMO_VARIO_MAX_MS = 8.0f;
const float DEMO_ALT_MIN_FT = 1.0f;
const float DEMO_TEMP_START_C = 23.0f;
const float DEMO_TEMP_MAX_C = 40.0f;
const float DEMO_TEMP_MIN_C = -5.0f;
const float DEMO_HUMIDITY_START = 25.0f;
const float DEMO_HUMIDITY_MAX = 90.0f;
const float DEMO_HUMIDITY_MIN = 30.0f;
const unsigned long DEMO_ICT_DURATION_MS = 70000UL;
const int DEMO_ICT_DURATION_SECONDS = 70;
static unsigned long demoStartMillis = 0;
// Clock simulation used only to test the sunset alert: starts 31 minutes before sunset.
static bool demoSunsetClockInitialized = false;
static bool demoSunsetShown = false;
static int demoSunsetBaseMinute = 0;
static unsigned long demoSunsetClockStartMillis = 0;
static bool demoIctInitialized = false;

// --- Traiettoria demo per le linee lubber ---
// L'aereo simulato gira su un cerchio di raggio DEMO_RADIUS_KM attorno a
// DEMO_BASE_LAT/LON. Se la home coincide con la base, la XTE rispetto alla
// corda decollo->casa oscilla tra -DEMO_RADIUS_KM e +DEMO_RADIUS_KM, cosi'
// si vedono tutti e tre i colori del corridoio (verde/giallo/rosso).
const float DEMO_BASE_LAT = 45.4642f;
const float DEMO_BASE_LON = 9.1900f;
const float DEMO_RADIUS_KM = 6.0f;         // > 3 km (verde) + 2 km (giallo) -> rosso
const float DEMO_CIRCLE_PERIOD_S = 90.0f;  // un giro ogni 90 s
static bool demoHomeAutoSet = false;       // true se la demo ha impostato la home da sola
// Traiettoria demo = un giro del cerchio, poi avvicinamento in linea retta a casa da 6 km
// (verso nord, a casa si ferma e la freccia lampeggia), poi si ricomincia.
const float DEMO_APPROACH_START_KM = 6.0f;
const float DEMO_APPROACH_SECONDS = 60.0f;
const float DEMO_HOME_HOLD_SECONDS = 10.0f;
// Dopo il giro: avvicinamento a Milano Linate in linea retta sull'asse della pista 35, in tempo reale:
// da 7 km a 1 km a 120 km/h (atterraggio sulla soglia), da 1 km frenata da 60 a 8 km/h fino al centro del campo,
// poi accelerazione fino a 300 km/h entro 2 km (decollo) e volo a 300 km/h fino a 10 km da Linate.
const float DEMO_LIN_A_S = 6.0f / 120.0f * 3600.0f;                 // 7 -> 1 km a 120 km/h (180 s)
const float DEMO_LIN_B_S = 1.0f / ((60.0f + 8.0f) / 2.0f) * 3600.0f; // 1 -> 0 km, da 60 a 8 km/h (106 s)
const float DEMO_LIN_C_S = 2.0f / ((8.0f + 300.0f) / 2.0f) * 3600.0f; // 0 -> 2 km, da 8 a 300 km/h (47 s)
const float DEMO_LIN_D_S = 8.0f / 300.0f * 3600.0f;                 // 2 -> 10 km a 300 km/h (96 s)
const float DEMO_LIN_SECONDS = DEMO_LIN_A_S + DEMO_LIN_B_S + DEMO_LIN_C_S + DEMO_LIN_D_S;
const float DEMO_LIN_ELEV_M = 107.0f;      // quota di Linate
const float DEMO_TRAJ_PERIOD_S = DEMO_CIRCLE_PERIOD_S + DEMO_LIN_SECONDS + DEMO_APPROACH_SECONDS + DEMO_HOME_HOLD_SECONDS;

// Avvicinamento demo a Linate. Posizione, rotta, quota (m), vario (m/s) e velocita' (km/h) al tempo ts.
static void demoLinateApproach(float ts, double &lat, double &lon, float &course, float &altM, float &vario, float &speed) {
  static int linIdx = -2;
  if (linIdx == -2) {   // cerca Linate nell'elenco dei campi (una volta sola)
    linIdx = -1;
    for (int i = 0; i < AIRFIELD_COUNT && linIdx < 0; ++i) {
      char up[40]; int j = 0;
      for (; AIRFIELDS[i].name[j] && j < 39; ++j) up[j] = (char)toupper((unsigned char)AIRFIELDS[i].name[j]);
      up[j] = '\0';
      if (strstr(up, "LINATE")) linIdx = i;
    }
  }
  double aLat = 45.4453, aLon = 9.2764;   // Linate (se non e' nell'elenco)
  float H = 356.0f, L = 2.44f;
  if (linIdx >= 0) {
    const AirfieldRec &f = AIRFIELDS[linIdx];
    aLat = f.lat5 / 100000.0; aLon = f.lon5 / 100000.0;
    if (f.rwLenM > 0) L = f.rwLenM / 1000.0f;
    if (f.rwy > 0) {   // l'estremita' della pista piu' vicina a 35
      const float a = f.rwy * 10.0f, b = fmodf(a + 180.0f, 360.0f);
      const float da = fabsf(fmodf(a - 350.0f + 540.0f, 360.0f) - 180.0f), db = fabsf(fmodf(b - 350.0f + 540.0f, 360.0f) - 180.0f);
      H = da <= db ? a : b;
    }
  }
  // Posizione lungo l'asse della pista (x in km dal centro del campo, positivo verso la prua H)
  const float ue = sinf(H * DEG_TO_RAD), un = cosf(H * DEG_TO_RAD);
  const float xThr = -L / 2.0f;                               // soglia: tocco a terra
  const float GS = tanf(3.0f * DEG_TO_RAD) * 1000.0f;         // sentiero di 3 gradi: metri per km
  float x, v;                                                 // km, km/h
  float t = ts;
  if (t < DEMO_LIN_A_S) {                                     // 7 -> 1 km a 120 km/h
    v = 120.0f; x = -7.0f + v * t / 3600.0f;
  } else if ((t -= DEMO_LIN_A_S) < DEMO_LIN_B_S) {            // da 1 km: da 60 a 8 km/h fino al campo
    const float a = (8.0f - 60.0f) / DEMO_LIN_B_S;            // km/h al secondo
    v = 60.0f + a * t; x = -1.0f + (60.0f * t + a * t * t / 2.0f) / 3600.0f;
  } else if ((t -= DEMO_LIN_B_S) < DEMO_LIN_C_S) {            // da 8 a 300 km/h in 2 km
    const float a = (300.0f - 8.0f) / DEMO_LIN_C_S;
    v = 8.0f + a * t; x = (8.0f * t + a * t * t / 2.0f) / 3600.0f;
  } else {                                                    // 300 km/h fino a 10 km
    t -= DEMO_LIN_C_S;
    if (t > DEMO_LIN_D_S) t = DEMO_LIN_D_S;
    v = 300.0f; x = 2.0f + v * t / 3600.0f;
  }
  // Quota sul campo: sentiero di 3 gradi fino alla soglia, a terra fino a 90 km/h in accelerazione,
  // poi salita di 8 gradi fino a 600 m.
  const float xLift = (90.0f * 90.0f - 8.0f * 8.0f) / (2.0f * (300.0f - 8.0f) / DEMO_LIN_C_S * 3600.0f);   // si stacca a 90 km/h
  float slope = 0.0f;                                         // metri di quota per km
  if (x < xThr) { altM = (xThr - x) * GS; slope = -GS; }
  else if (x <= xLift || ts < DEMO_LIN_A_S + DEMO_LIN_B_S) { altM = 0.0f; }
  else {
    const float CL = tanf(8.0f * DEG_TO_RAD) * 1000.0f;
    altM = (x - xLift) * CL; slope = CL;
    if (altM > 600.0f) { altM = 600.0f; slope = 0.0f; }
  }
  vario = slope / 1000.0f * v / 3.6f;
  speed = v;
  const float e = x * ue, n = x * un;
  lat = aLat + n / 111.0;
  lon = aLon + e / (111.0 * cos(aLat * DEG_TO_RAD));
  course = H;
  altM += DEMO_LIN_ELEV_M;
}

void updateDemoData() {
  if (!demoMode || sensorDataMutex == nullptr) return;
  const unsigned long now = millis();
  // In demo si finge anche il meteo dal telefono, cosi' tutto si vede senza avere l'app.
  // I valori girano piano, con un giro completo in due minuti: la freccia del vento ruota
  // davvero (e con lei il pallino e il punto da toccare), il vento passa dal verde al rosso
  // lampeggiante, e la pagina ONLINE WEATHER ha tutte e sei le righe piene.
  // Fuori dalla demo questi valori li scrive solo il Bluetooth, e lo stop demo li azzera.
  {
    const float ph = (float)(now % 120000UL) / 120000.0f * 6.2831853f;
    const float s1 = sinf(ph), s2 = sinf(ph * 2.0f);
    metQnh = 1013.0f + 4.0f * s1;
    int wd = (int)(240.0f + 70.0f * s1) % 360;
    if (wd < 0) wd += 360;
    metWindDir = wd;
    metWindKt = 6.0f + 11.0f * (0.5f + 0.5f * s2);                  // da 6 a 17 nodi
    metGustKt = metWindKt + 2.0f + 9.0f * (0.5f + 0.5f * s1);       // raffica: a tratti scatta il rosso
    metQnhValid = true; metWindValid = true;
    metVisM = (int)(1500.0f + 9000.0f * (0.5f + 0.5f * sinf(ph + 1.0f)));
    metVisValid = true;
    metPrecipProb = (int)(10.0f + 70.0f * (0.5f + 0.5f * sinf(ph + 2.0f)));
    metPrecipMm = metPrecipProb > 50 ? 0.6f : 0.0f;
    metPrecipValid = true;
    metFrzM = (int)(1600.0f + 900.0f * s1);
    metFrzValid = true;
    metDewC = 6.0f + 4.0f * s1;
    metDewValid = true;
    metPhoneLat = DEMO_BASE_LAT; metPhoneLon = DEMO_BASE_LON; metPhonePosValid = true;
    metAtMs = now;
  }
  if (demoStartMillis == 0) demoStartMillis = now;
  if (demoIntroActive) demoStartMillis = now;   // sequenza iniziale in corso: la routine demo parte alla fine
  const float t = (float)(now - demoStartMillis) / 1000.0f;
  const float phase = fmodf(t, DEMO_CYCLE_SECONDS);
  const float rampFraction = (phase < DEMO_HALF_CYCLE_SECONDS)
      ? phase / DEMO_HALF_CYCLE_SECONDS
      : (phase - DEMO_HALF_CYCLE_SECONDS) / DEMO_HALF_CYCLE_SECONDS;

  const float demoVario = (phase < DEMO_HALF_CYCLE_SECONDS)
      ? DEMO_VARIO_MIN_MS + (DEMO_VARIO_MAX_MS - DEMO_VARIO_MIN_MS) * rampFraction
      : DEMO_VARIO_MAX_MS - (DEMO_VARIO_MAX_MS - DEMO_VARIO_MIN_MS) * rampFraction;
  // Quota coerente con il vario: e' l'integrale della rampa del vario (sale quando il vario e'
  // positivo, scende quando e' negativo). Il punto piu' basso vale DEMO_ALT_MIN_FT.
  // Con vario -8..+8 m/s e mezzo ciclo di 60 s la quota oscilla di 240 m (circa 1-790 ft).
  const float vSpan = DEMO_VARIO_MAX_MS - DEMO_VARIO_MIN_MS;
  const float H = DEMO_HALF_CYCLE_SECONDS;
  const float p0 = -DEMO_VARIO_MIN_MS * H / vSpan;                         // istante con vario = 0
  const float minIntegral = DEMO_VARIO_MIN_MS * p0 + vSpan * p0 * p0 / (2.0f * H);
  float integral;
  if (phase < H) {
    integral = DEMO_VARIO_MIN_MS * phase + vSpan * phase * phase / (2.0f * H);
  } else {
    const float q = phase - H;
    integral = DEMO_VARIO_MAX_MS * q - vSpan * q * q / (2.0f * H);
  }
  const float demoAlt = DEMO_ALT_MIN_FT * 0.3048f - minIntegral + integral;  // metri (GPS/sensori interni)
  const float demoSpeed = (phase < DEMO_HALF_CYCLE_SECONDS)
      ? DEMO_SPEED_MIN_KMH + (DEMO_SPEED_MAX_KMH - DEMO_SPEED_MIN_KMH) * rampFraction
      : DEMO_SPEED_MAX_KMH - (DEMO_SPEED_MAX_KMH - DEMO_SPEED_MIN_KMH) * rampFraction;

  float demoTemp;
  float demoHumidity;
  if (phase < DEMO_HALF_CYCLE_SECONDS) {
    demoTemp = DEMO_TEMP_START_C + (DEMO_TEMP_MAX_C - DEMO_TEMP_START_C) * rampFraction;
    demoHumidity = DEMO_HUMIDITY_START +
                   (DEMO_HUMIDITY_MAX - DEMO_HUMIDITY_START) * rampFraction;
  } else {
    demoTemp = DEMO_TEMP_MAX_C + (DEMO_TEMP_MIN_C - DEMO_TEMP_MAX_C) * rampFraction;
    demoHumidity = DEMO_HUMIDITY_MAX +
                   (DEMO_HUMIDITY_MIN - DEMO_HUMIDITY_MAX) * rampFraction;
  }

  // In demo l'ICT è sempre attivo e scade ogni 70 s; l'impostazione normale
  // dell'utente viene salvata/ripristinata all'uscita dalla demo.
  if (!demoIctInitialized) {
    ictEnabled = true;
    ictStartMillis = demoStartMillis;
    demoIctInitialized = true;
    if (currentScreen == ICT_SCREEN) drawIctScreen();
  }
  ictEnabled = true;
  if (demoIntroActive) ictStartMillis = demoStartMillis;   // i 70 s dell'ICT contano dalla fine della sequenza

  float simAlt = demoAlt, simVario = demoVario, simSpeed = demoSpeed;   // l'avvicinamento a Linate li sostituisce
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
    gpsRmcStatusValid = true;
    gpsDataValid = true;
    gps_fix_quality = 1;
    gps_satellites = 8;
    // Traiettoria circolare ampia: la corda decollo->casa interseca il cerchio
    // in due punti, quindi la XTE oscilla con ampiezza pari al raggio e le
    // linee lubber mostrano verde (dentro), giallo (margine) e rosso (fuori).
    static int demoPrevSeg = 0;   // 0 cerchio, 2 avvicinamento e atterraggio a Linate, 1 avvicinamento a casa
    const float tt = fmodf(t, DEMO_TRAJ_PERIOD_S);
    const int demoSeg = (tt < DEMO_CIRCLE_PERIOD_S) ? 0 : (tt < DEMO_CIRCLE_PERIOD_S + DEMO_LIN_SECONDS) ? 2 : 1;
    if (demoSeg == 2) {
      double la, lo; float c;
      demoLinateApproach(tt - DEMO_CIRCLE_PERIOD_S, la, lo, c, simAlt, simVario, simSpeed);
      gps_latitude = (float)la; gps_longitude = (float)lo; gps_course = c;
    } else if (demoSeg == 0) {
      const float demoAngle = tt * (TWO_PI / DEMO_CIRCLE_PERIOD_S);
      const float demoNorthKm = DEMO_RADIUS_KM * cosf(demoAngle);
      const float demoEastKm  = DEMO_RADIUS_KM * sinf(demoAngle);
      const float demoBaseLatRad = DEMO_BASE_LAT * DEG_TO_RAD;
      gps_latitude  = DEMO_BASE_LAT + demoNorthKm / 111.0f;
      gps_longitude = DEMO_BASE_LON + demoEastKm  / (111.0f * cosf(demoBaseLatRad));
      // Course tangente al cerchio: coincide con la direzione di moto reale.
      gps_course = fmodf(90.0f + tt * (360.0f / DEMO_CIRCLE_PERIOD_S), 360.0f);
    } else {
      // Avvicinamento: parte 6 km a sud di casa e va dritto verso nord fino a casa, poi resta li'.
      float f = (tt - DEMO_CIRCLE_PERIOD_S - DEMO_LIN_SECONDS) / DEMO_APPROACH_SECONDS;
      if (f > 1.0f) f = 1.0f;
      const double hLat = homePositionValid ? homeLatitude : (double)DEMO_BASE_LAT;
      const double hLon = homePositionValid ? homeLongitude : (double)DEMO_BASE_LON;
      // Leggera serpentina (3 oscillazioni, ampiezza 400 m che si riduce fino a 0 su casa): la rotta
      // cambia continuamente, cosi' la freccia (grande e poi quella piccola nel bersaglio) si muove.
      const float wA = 0.4f, wK = 3.0f * TWO_PI;
      const float eastKm = wA * (1.0f - f) * sinf(wK * f);
      gps_latitude  = hLat - (double)(DEMO_APPROACH_START_KM * (1.0f - f)) / 111.0;
      gps_longitude = hLon + (double)eastKm / (111.0 * cos(hLat * DEG_TO_RAD));
      if (f < 1.0f) {
        const float dNorth = DEMO_APPROACH_START_KM;                                   // km per unita' di f
        const float dEast = wA * (-sinf(wK * f) + (1.0f - f) * wK * cosf(wK * f));
        float c = atan2f(dEast, dNorth) * RAD_TO_DEG;
        if (c < 0.0f) c += 360.0f;
        gps_course = c;
      } else {
        gps_course = 0.0f;
      }
    }
    if (demoSeg != demoPrevSeg) {
      // Nuova tratta: il corridoio lubber riparte dal punto attuale e torna attivo.
      lubberLegLat = gps_latitude;
      lubberLegLon = gps_longitude;
      lubberLegValid = true;
      lubberActive = true;
      demoPrevSeg = demoSeg;
    }
    gps_altitude = simAlt;
    gps_ground_speed_kmh = simSpeed;
    gps_utc_hours = 12;
    gps_utc_minutes = (int)(fmodf(t, 3600.0f) / 60.0f);
    gps_utc_seconds = (int)fmodf(t, 60.0f);
    gps_utc_day = 4;
    gps_utc_month = 9;
    gps_utc_year = 2026;
    gps_fix_type = 3;
    gps_has_gps = true;
    gps_has_glonass = true;
    gps_has_galileo = true;
    gps_has_bds = true;
    gps_has_qzss = true;
    gps_last_gsv_gps = now;
    gps_last_gsv_glonass = now;
    gps_last_gsv_galileo = now;
    gps_last_gsv_bds = now;
    gps_last_gsv_qzss = now;
    lastGpsMessageMillis = now;
    sharedFilteredAlt = simAlt;
    sharedAltitudeValid = true;
    sharedVarioValid = true;
    sharedVarioKalman = simVario;
    meteo_temperature_c = demoTemp;
    meteo_humidity_percent = demoHumidity;
    meteoValid = true;
    if (simSpeed > maxGpsSpeedKmh) maxGpsSpeedKmh = simSpeed;
    if (simVario > maxClimbRate) maxClimbRate = simVario;
    if (simVario < maxSinkRate) maxSinkRate = simVario;
    xSemaphoreGive(sensorDataMutex);
    syncSoftwareClockFromGps(gps_utc_hours, gps_utc_minutes, gps_utc_seconds,
                             gps_utc_day, gps_utc_month, gps_utc_year);
  }
}

void sensorTask(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(100);
  xLastWakeTime = xTaskGetTickCount();
  esp_task_wdt_add(NULL);
  while (true) {
    esp_task_wdt_reset();
    if (demoMode || bleRole == BLE_ROLE_SLAVE || !bmeFoundAtBoot) {   // slave / strumento senza sensori: dati dal master (BLE) o assenti
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }
    if (!bmeReady) {
      unsigned long now = millis();
      if (now - lastBmeRecoveryAttemptMillis >= BME_RECOVERY_RETRY_MS) {
        lastBmeRecoveryAttemptMillis = now;
        bool gotBus = (i2cMutex == nullptr) ||
                      (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(50)) == pdTRUE);
        if (gotBus) {
          if (recoverBME280Locked()) resetVarioAfterBMERecovery();
          if (i2cMutex != nullptr) xSemaphoreGive(i2cMutex);
        }
      }
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      continue;
    }

    bool gotBus = (i2cMutex == nullptr) ||
                  (xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(20)) == pdTRUE);
    float rawP = NAN, rawAlt = NAN, rawTemperature = NAN, rawHumidity = NAN;
    bool validReading = false;
    bool recoveredNow = false;
    const char *invalidReason = "pressure out of range or non-finite";

    if (gotBus) {
      const float localQnh = constrain((float)seaLevelPressure, QNH_MIN_HPA, QNH_MAX_HPA);
      rawP = sensor_bme.readPressure() / 100.0F;
      if (isfinite(rawP) && rawP >= BME_MIN_PRESSURE_HPA && rawP <= BME_MAX_PRESSURE_HPA) {
        // Stessa formula di readAltitude(), ma dalla pressione appena letta (niente
        // seconda lettura sul bus). altOffsetM: offset di taratura della quota (pagina OFFSET, default +11 m).
        const float offsetNow = altOffsetM;
        static float offsetUsed = NAN;
        if (offsetNow != offsetUsed) {   // offset cambiato: il gradino non e' una lettura errata
          if (isfinite(offsetUsed)) { lastAcceptedRawAlt = NAN; offsetChangedReset = true; }
          offsetUsed = offsetNow;
        }
        rawAlt = (float)(44330.0 * (1.0 - pow((double)rawP / (double)localQnh, 0.1903))) + offsetNow;
        rawTemperature = sensor_bme.readTemperature();
        rawHumidity = sensor_bme.readHumidity();
        const bool qnhChangeGrace = qnhChangedAtMillis != 0 &&
          millis() - qnhChangedAtMillis <= QNH_CHANGE_GRACE_MS;
        // [FIX A] Il gradino viene valutato rispetto all'ultima lettura
        // grezza accettata, non all'altitudine filtrata (che puo' essere
        // momentaneamente indietro se il loop e' occupato in un redraw).
        const bool stepCheckOk = !isfinite(lastAcceptedRawAlt) ||
                                 fabsf(rawAlt - lastAcceptedRawAlt) <= BME_MAX_ALTITUDE_STEP_M;
        if (!isfinite(rawAlt) || rawAlt <= -1000.0f || rawAlt >= 20000.0f) {
          invalidReason = "altitude out of range or non-finite";
        } else if (!qnhChangeGrace && !stepCheckOk) {
          invalidReason = "altitude step too large";
        } else if (!isfinite(rawTemperature) || rawTemperature <= -50.0f || rawTemperature >= 100.0f) {
          invalidReason = "temperature out of range or non-finite";
        } else if (!isfinite(rawHumidity) || rawHumidity < 0.0f || rawHumidity > 100.0f) {
          invalidReason = "humidity out of range or non-finite";
        } else {
          validReading = true;
        }
      }

      if (validReading) {
        // [FIX A] memorizza l'ultima lettura grezza accettata
        lastAcceptedRawAlt = rawAlt;
        bmeConsecutiveErrors = 0;
      } else if (++bmeConsecutiveErrors >= BME_MAX_CONSECUTIVE_ERRORS) {
        const float deltaAltitude = (isfinite(rawAlt) && isfinite(lastAcceptedRawAlt))
                                      ? rawAlt - lastAcceptedRawAlt : NAN;
        AVEA_LOGF("BME280: lettura non valida (%s): P=%.2f hPa, quota=%.2f m, "
                      "T=%.2f C, RH=%.2f%%, deltaQuota=%.2f m, QNH=%.2f hPa; avvio recovery\n",
                      invalidReason, rawP, rawAlt, rawTemperature, rawHumidity,
                      deltaAltitude, localQnh);
        lastBmeRecoveryAttemptMillis = millis();
        recoveredNow = recoverBME280Locked();
      }
      if (i2cMutex != nullptr) xSemaphoreGive(i2cMutex);
    }

    if (recoveredNow) resetVarioAfterBMERecovery();
    if (validReading) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rawAltitudeShared = rawAlt;
        rawAltitudeValid = true;
        newAltitudeSample = true;
        if (offsetChangedReset) { varioResetRequested = true; offsetChangedReset = false; }   // filtro ripartito con il nuovo offset
        lastBmeValidMillis = millis();
        meteo_temperature_c = rawTemperature;
        meteo_humidity_percent = rawHumidity;
        meteoValid = true;
        xSemaphoreGive(sensorDataMutex);
      }
    }
    if (lastBmeValidMillis != 0 && millis() - lastBmeValidMillis > BME_DATA_TIMEOUT_MS) {
      if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        rawAltitudeValid = false;
        sharedAltitudeValid = false;
        sharedVarioValid = false;
        meteoValid = false;
        xSemaphoreGive(sensorDataMutex);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ========== FUNZIONI DI DISEGNO ==========

// [FIX QUADRATINI] Versione precedente: offsettava N linee di 1 px lungo la
// perpendicolare, lasciando buchi sulle diagonali. Ora disegna il
// quadrilatero pieno con due triangoli: nessun buco.
void drawThickLine(int x0, int y0, int x1, int y1, int color, int thickness) {
  if (thickness <= 1) { lcd.drawLine(x0, y0, x1, y1, color); return; }
  const float dx = (float)(x1 - x0);
  const float dy = (float)(y1 - y0);
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) { lcd.fillCircle(x0, y0, thickness / 2, color); return; }

  const float nx = -dy / len;
  const float ny =  dx / len;
  const float half = thickness * 0.5f;

  const int ax = (int)lroundf(x0 + nx * half);
  const int ay = (int)lroundf(y0 + ny * half);
  const int bx = (int)lroundf(x0 - nx * half);
  const int by = (int)lroundf(y0 - ny * half);
  const int cx = (int)lroundf(x1 + nx * half);
  const int cy = (int)lroundf(y1 + ny * half);
  const int ex = (int)lroundf(x1 - nx * half);
  const int ey = (int)lroundf(y1 - ny * half);

  lcd.fillTriangle(ax, ay, bx, by, cx, cy, color);
  lcd.fillTriangle(bx, by, cx, cy, ex, ey, color);
}

// Stessa linea spessa di drawThickLine (due triangoli) su qualunque superficie (lcd o sprite):
// usata per le tacche e i trattini degli archi, cosi' hanno lo stesso spessore dell'ALTIMETER.
void drawThickLineOn(lgfx::LGFXBase &g, int x0, int y0, int x1, int y1, int color, int thickness) {
  if (thickness <= 1) { g.drawLine(x0, y0, x1, y1, color); return; }
  const float dx = (float)(x1 - x0), dy = (float)(y1 - y0);
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) { g.fillCircle(x0, y0, thickness / 2, color); return; }
  const float nx = -dy / len, ny = dx / len, half = thickness * 0.5f;
  const int ax = (int)lroundf(x0 + nx * half), ay = (int)lroundf(y0 + ny * half);
  const int bx = (int)lroundf(x0 - nx * half), by = (int)lroundf(y0 - ny * half);
  const int cx = (int)lroundf(x1 + nx * half), cy = (int)lroundf(y1 + ny * half);
  const int ex = (int)lroundf(x1 - nx * half), ey = (int)lroundf(y1 - ny * half);
  g.fillTriangle(ax, ay, bx, by, cx, cy, color);
  g.fillTriangle(bx, by, cx, cy, ex, ey, color);
}

// [FIX QUADRATINI] Stessa correzione per la versione sprite.
void drawThickLineSprite(LGFX_Sprite &sprite, int x0, int y0, int x1, int y1, int color, int thickness) {
  if (thickness <= 1) { sprite.drawLine(x0, y0, x1, y1, color); return; }
  const float dx = (float)(x1 - x0);
  const float dy = (float)(y1 - y0);
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) { sprite.fillCircle(x0, y0, thickness / 2, color); return; }

  const float nx = -dy / len;
  const float ny =  dx / len;
  const float half = thickness * 0.5f;

  const int ax = (int)lroundf(x0 + nx * half);
  const int ay = (int)lroundf(y0 + ny * half);
  const int bx = (int)lroundf(x0 - nx * half);
  const int by = (int)lroundf(y0 - ny * half);
  const int cx = (int)lroundf(x1 + nx * half);
  const int cy = (int)lroundf(y1 + ny * half);
  const int ex = (int)lroundf(x1 - nx * half);
  const int ey = (int)lroundf(y1 - ny * half);

  sprite.fillTriangle(ax, ay, bx, by, cx, cy, color);
  sprite.fillTriangle(bx, by, cx, cy, ex, ey, color);
}

void drawThickTriangleSprite(LGFX_Sprite &sprite, int x1, int y1,
                             int x2, int y2, int x3, int y3,
                             int color, int thickness) {
  // Riempie il triangolo e poi ridisegna il bordo spesso.
  sprite.fillTriangle(x1, y1, x2, y2, x3, y3, color);
  drawThickLineSprite(sprite, x1, y1, x2, y2, color, thickness);
  drawThickLineSprite(sprite, x2, y2, x3, y3, color, max(1, thickness - 1));
  drawThickLineSprite(sprite, x3, y3, x1, y1, color, thickness);
}

// Tacche dei gradi delle bussole (COMPASS e GO HOME): spessore in pixel, modificabile qui.
#define COMPASS_TICK_THICKNESS_PX 3
void drawTickLine15Sprite(LGFX_Sprite &sprite, int x0, int y0, int x1, int y1,
                          uint16_t color, uint16_t bg) {
  (void)bg;
  drawThickLineSprite(sprite, x0, y0, x1, y1, color, COMPASS_TICK_THICKNESS_PX);
}

void drawAirplaneSprite(LGFX_Sprite &sprite, int cx, int cy, int color, float scale = 1.0f) {
  // Usa esclusivamente il bitmap originale: il contorno e la sagoma restano
  // identici. Poi riempie ogni riga tra il primo e l'ultimo pixel del contorno.
  (void)scale;
  drawAirplaneBitmap(sprite, cx, cy, color);

  // AirplaneBitmap.h definisce una sagoma centrata di 115x112 pixel.
  const int left = cx - 57;
  const int top = cy - 56;
  for (int y = top; y < top + 112; ++y) {
    int first = -1;
    int last = -1;
    for (int x = left; x < left + 115; ++x) {
      if (sprite.readPixel(x, y) == color) {
        if (first < 0) first = x;
        last = x;
      }
    }
    if (first >= 0 && last > first) {
      sprite.drawFastHLine(first, y, last - first + 1, color);
    }
  }
}

// Alloca (una sola volta) il framebuffer 360x360 di Bussola e Go To Home.
// Ordine dei tentativi: PSRAM 16 bit -> RAM interna 16 bit -> RAM interna 8 bit
// (RGB332, meta' della memoria, colori leggermente meno ricchi ma funziona).
// Se fallisce riprova ogni 3 s: la RAM interna si libera quando si chiude il WiFi.
static bool ensureNavigationSprite() {
  if (navigationSpriteReady) return true;
  static unsigned long lastTryMs = 0;
  if (navigationSpriteAllocationFailed && millis() - lastTryMs < 3000UL) return false;
  lastTryMs = millis();
  navigationSprite.deleteSprite();
  const int depths[3] = {16, 16, 8};
  for (int attempt = 0; attempt < 3; ++attempt) {
    if (attempt == 0 && !psramFound()) continue;
    navigationSprite.setPsram(attempt == 0);
    navigationSprite.setColorDepth(depths[attempt]);
    navigationSprite.createSprite(360, 360);
    if (navigationSprite.getBuffer() != nullptr) {
      navigationSpriteReady = true;
      navigationSpriteAllocationFailed = false;
      AVEA_LOGF("[NAV] sprite OK: tentativo %d, %d bit, %s\n", attempt, depths[attempt], attempt == 0 ? "PSRAM" : "RAM interna");
      return true;
    }
    navigationSprite.deleteSprite();
  }
  navigationSpriteAllocationFailed = true;
  AVEA_LOGF("[NAV] sprite FALLITO: heap libero=%u, blocco max=%u, PSRAM=%s\n",
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), psramFound() ? "si" : "NO");
  return false;
}

void drawCompassScreen(bool forceRedraw) {
  CompassThemeScope compassThemeScope;   // il tema (bianco/nero, giallo/nero) vale solo qui
  float course = 0.0f, speedKmh = 0.0f;
  bool gpsValid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    course = gps_course;
    speedKmh = gps_ground_speed_kmh;
    gpsValid = gpsDataValid;
    xSemaphoreGive(sensorDataMutex);
  }
  const bool courseUsable = gpsValid && speedKmh >= GS_ZERO_THRESHOLD_KMH;
  if (gpsValid && !courseUsable && lastPrintedCompassCourse > -360.0f) {
    course = lastPrintedCompassCourse;
  }
  bool stateChanged = (gpsValid != lastGpsValid);
  if (stateChanged) forceRedraw = true;
  bool useRed = !gpsValid;
  uint16_t bgColor = themeBg();
  uint16_t redColor = themeColor(TFT_RED);
  uint16_t aircraftColor = themeColor(TFT_YELLOW);
  uint16_t tickColor = useRed ? redColor : themeColor(TFT_WHITE);
  uint16_t numColor = useRed ? redColor : themeColor(TFT_WHITE);
  if (!forceRedraw && gpsValid == lastGpsValid) {
    float courseDelta = fabsf(course - lastPrintedCompassCourse);
    if (courseDelta > 180.0f) courseDelta = 360.0f - courseDelta;
    if (courseDelta < 0.3f) return;
  }

  lastPrintedCompassCourse = course;
  lastPrintedCompassSpeed = speedKmh;
  lastGpsValid = gpsValid;

  if (!ensureNavigationSprite()) {
    lcd.fillScreen(bgColor);
    lcd.setTextDatum(MC_DATUM);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize(1, 1);
    lcd.setTextColor(redColor, bgColor);
    lcd.drawString("MEMORIA INSUFFICIENTE", 180, 160);
    lcd.drawString("BUSSOLA NON DISPONIBILE", 180, 200);
    return;
  }
  navigationSprite.fillSprite(bgColor);
  navigationSprite.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  const int cx = 180, cy = 180;
  const int tick_radius = 175;
  const int num_radius = 145;
  const float ref_angle = gpsValid ? course : 0.0f;
  for (int angle = 0; angle < 360; angle += 10) {
    const float rad = (angle - ref_angle - 90) * DEG_TO_RAD;
    const int x_outer = cx + tick_radius * cos(rad);
    const int y_outer = cy + tick_radius * sin(rad);
    if (angle % 30 == 0) {
      const int x_inner = cx + (tick_radius - 20) * cos(rad);
      const int y_inner = cy + (tick_radius - 20) * sin(rad);
      drawTickLine15Sprite(navigationSprite, x_inner, y_inner, x_outer, y_outer, tickColor, bgColor);
      const int num_x = cx + num_radius * cos(rad);
      const int num_y = cy + num_radius * sin(rad);
      navigationSprite.setTextDatum(MC_DATUM);
      navigationSprite.setTextColor(numColor, bgColor);
      if (angle == 0 || angle == 90 || angle == 180 || angle == 270) {
        navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
        navigationSprite.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
        const char* cardinal = angle == 0 ? "N" : (angle == 90 ? "E" : (angle == 180 ? "S" : "W"));
        navigationSprite.setTextColor(angle == 0 ? themeColor(TFT_ORANGE) : numColor, bgColor);
        navigationSprite.drawString(cardinal, num_x, num_y);
      }
      else {
        char buf[4];
        snprintf(buf, sizeof(buf), "%d", angle / 10);
        drawNumericVlwSprite(navigationSprite, buf, num_x, num_y, MC_DATUM, numColor, bgColor, 0.34f);
      }
    } else {
      const int x_inner = cx + (tick_radius - 10) * cos(rad);
      const int y_inner = cy + (tick_radius - 10) * sin(rad);
      drawTickLine15Sprite(navigationSprite, x_inner, y_inner, x_outer, y_outer, tickColor, bgColor);
    }
  }

  const uint16_t compassCyan = themeColor(TFT_CYAN);
  drawThickTriangleSprite(navigationSprite, cx, 7, cx - 8, 27, cx + 8, 27,
                          compassCyan, 3);
  drawThickTriangleSprite(navigationSprite, cx, 353, cx - 8, 333, cx + 8, 333,
                          compassCyan, 3);
  // Aereo + rotta centrati nel display: il gruppo (aereo sopra, gradi sotto) e' centrato
  // verticalmente su cy; il numero con il ° e' centrato in orizzontale come gruppo.
  const int centerShift = 13;  // far salire gradi
  drawAirplaneSprite(navigationSprite, cx, cy - 60 + centerShift, aircraftColor, 1.05f);  // far calire aereo

  // Riquadro con la scritta A.V.E.A (solo scritta: niente tocco, la pagina COORDS e' stata eliminata).
  {
    // Il riquadro sta a 3 px dai bordi reali della scritta: l'altezza dei glifi si misura una volta
    // sola sullo sprite (dopo il primo disegno) e si riusa nei ridisegni successivi.
    const int coordsCy = 280, coordsPad = 3;
    static int coordsTop = -1, coordsBot = -1;
    const uint32_t bgPx = navigationSprite.readPixelValue(125, coordsCy);
    const int coordsW = drawUiVlwSprite(navigationSprite, "A.V.E.A*", 180, coordsCy, MC_DATUM, compassCyan, bgColor, 0.62f);
    if (coordsTop < 0) {
      int top = 10000, bot = -1;
      for (int y = coordsCy - 16; y <= coordsCy + 16; ++y) {
        for (int x = 180 - coordsW / 2 - 2; x <= 180 + coordsW / 2 + 2; ++x) {
          if (navigationSprite.readPixelValue(x, y) != bgPx) {
            if (y < top) top = y;
            if (y > bot) bot = y;
            break;
          }
        }
      }
      if (bot >= top && bot - top >= 5 && bot - top <= 30) { coordsTop = top; coordsBot = bot; }
    }
    const int boxTop = (coordsTop >= 0 ? coordsTop : coordsCy - 13) - coordsPad - 1;   // 3 px liberi + riga del bordo
    const int boxBot = (coordsBot >= 0 ? coordsBot : coordsCy + 13) + coordsPad + 1;
    const int boxW = max(100, coordsW + 16);   // riquadro largo almeno quanto la scritta
    navigationSprite.drawRect(180 - boxW / 2, boxTop, boxW, boxBot - boxTop + 1, compassCyan);
  }

  if (gpsValid) {
    const int courseValue = (int)round(course);
    char courseBuf[12];
    snprintf(courseBuf, sizeof(courseBuf), "%d", courseValue);
    const uint16_t courseColor = useRed ? redColor : themeColor(TFT_YELLOW);
    const int numY = cy + 35 + centerShift;
    const int numX = cx - 10;  // compensa l'ingombro del simbolo ° (circa 19 px) a destra del numero
    const int courseWidth = drawNumericVlwSprite(navigationSprite, courseBuf, numX, numY, MC_DATUM, useRed ? redColor : TFT_YELLOW, bgColor, 0.72f);
    const int degreeX = numX + (courseWidth / 2) + 14;
    const int degreeY = numY - 16;
    navigationSprite.drawCircle(degreeX, degreeY, 5, courseColor);
    navigationSprite.drawCircle(degreeX, degreeY, 4, courseColor);
  } else {
    navigationSprite.setTextDatum(MC_DATUM);
    navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
    navigationSprite.setTextSize((1.35f) * THEME_TEXT_SCALE, (1.35f) * THEME_TEXT_SCALE);
    navigationSprite.setTextColor(redColor, bgColor);
    navigationSprite.drawString("NO GPS FIX", cx, cy + 35 + centerShift);
  }
  navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

void saveScreenPreference() {
  if (demoTourActive) return;   // rotazione demo: nessuna scrittura
  int activeIndex = -1;
  for (int i = 0; i < numScreens; ++i) {
    if (screenOrder[i] == currentScreen) {
      activeIndex = i;
      break;
    }
  }
  if (activeIndex < 0) return;
  currentScreenIndex = activeIndex;
  preferences.begin("altimeter", false);
  if (preferences.getInt("bootScreen", -1) != (int)currentScreen) {
    preferences.putInt("bootScreen", (int)currentScreen);
  }
  if (preferences.getInt("lastScreen", -1) != activeIndex) {
    preferences.putInt("lastScreen", activeIndex);
  }
  preferences.end();
}

void loadScreenPreference() {
  loadPagesMask();
  loadVerticalSwipeSetting();
  loadAltOffset();
  loadFuelSettings();
  preferences.begin("altimeter", true);
  const int savedScreen = preferences.getInt("bootScreen", -1);
  int idx = -1;
  for (int i = 0; i < numScreens; ++i) {
    if ((int)screenOrder[i] == savedScreen) {
      idx = i;
      break;
    }
  }
  // Se la pagina salvata non e' piu' nel giro (logo, manual flight timer, filter)
  // si riparte dalla prima pagina; "lastScreen" vale solo se manca bootScreen.
  if (idx < 0) idx = (savedScreen < 0) ? preferences.getInt("lastScreen", 0) : 0;
  if (idx < 0 || idx >= numScreens) idx = 0;
  if (!pageIndexEnabled(idx)) idx = nextEnabledScreenIndex(idx, 1);   // pagina salvata disattivata: prima attiva
  currentScreenIndex = idx;
  currentScreen = screenOrder[idx];
  preferences.end();
}

void saveDisplayThemePreference() {
  int theme = (int)compassTheme;
  if (theme < (int)THEME_COLOR || theme > (int)THEME_YELLOW_BLACK) {
    theme = (int)THEME_COLOR;
  }
  if (preferences.begin("display", false)) {
    preferences.putUChar("theme", (uint8_t)theme);
    preferences.end();
  }
}

void loadDisplayThemePreference() {
  int savedTheme = (int)THEME_COLOR;
  bool found = false;

  if (preferences.begin("display", true)) {
    if (preferences.isKey("theme")) {
      savedTheme = (int)preferences.getUChar("theme", (uint8_t)THEME_COLOR);
      found = true;
    }
    preferences.end();
  }

  if (!found && preferences.begin("altimeter", true)) {
    if (preferences.isKey("display_theme")) {
      savedTheme = preferences.getInt("display_theme", (int)THEME_COLOR);
      found = true;
    }
    preferences.end();
    if (found) saveDisplayThemePreference();
  }

  if (savedTheme < (int)THEME_COLOR || savedTheme > (int)THEME_YELLOW_BLACK) {
    savedTheme = (int)THEME_COLOR;
  }
  compassTheme = (DisplayTheme)savedTheme;   // solo la bussola usa il tema salvato
  displayTheme = THEME_COLOR;                // le altre pagine sono sempre a colori
}

// ---- Barre variometro sul lato sinistro della schermata altimetro ----
// Una barra per ogni m/s (fondo scala +-11 m/s), sempre piu spesse: 1 m/s la piu sottile, 11 m/s la piu spessa.
// Partono dal centro del bordo sinistro (zero) e salgono in salita, scendono in discesa.
const int ALT_VARIO_BARS = 5;
const float ALT_VARIO_SPAN_DEG = 60.0f;   // arco totale per lato
const float ALT_VARIO_GAP_DEG = 1.5f;     // spazio tra le barre
const int ALT_VARIO_R_OUT = 172;          // bordo esterno fisso, le barre crescono verso il centro
const float ALT_VARIO_T_MIN = 17.0f;      // spessore uguale alle tacche di destra (172 - 155 px)
const int ALT_VARIO_ZERO_X = 2;           // linea bianca dello zero
const int ALT_VARIO_ZERO_W = 18;
const int ALT_VARIO_TEXT_X = ALT_VARIO_ZERO_X + ALT_VARIO_ZERO_W + 10;   // valore a 10 px dalla linea bianca

// Superficie su cui disegnano fillArcBar / barre vario (lcd di norma; lo sprite per la pagina QFE).
static lgfx::LGFXBase *arcTarget = &lcd;
// Settore d'anello con angoli in gradi misurati dal punto 9 (bordo sinistro), positivi verso l'alto.
// Con rightSide=true gli angoli sono misurati dal punto 3 (bordo destro).
const float ARC_R_OUT = 177.5f;   // bordo esterno delle fasce degli archi (centro 179.5): ~2 px di nero dal vetro
const float ARC_TICK_R = 176.0f;  // tacche della zona 0-65 km/h: fin dentro l'arco bianco
static void fillArcBar(int cx, int cy, float rIn, float rOut, float a0, float a1, uint16_t color, bool rightSide = false) {
  int steps = (int)ceilf((a1 - a0) / 3.0f);
  if (steps < 1) steps = 1;
  const float da = (a1 - a0) / (float)steps;
  auto pt = [&](float r, float deg, int &x, int &y) {
    const float a = deg * DEG_TO_RAD;
    x = cx + (rightSide ? 1 : -1) * (int)lroundf(r * cosf(a));
    y = cy - (int)lroundf(r * sinf(a));
  };
  int ox0, oy0, ix0, iy0;
  pt(rOut, a0, ox0, oy0);
  pt(rIn, a0, ix0, iy0);
  for (int k = 0; k < steps; k++) {
    int ox1, oy1, ix1, iy1;
    const float a = a0 + da * (float)(k + 1);
    pt(rOut, a, ox1, oy1);
    pt(rIn, a, ix1, iy1);
    arcTarget->fillTriangle(ox0, oy0, ox1, oy1, ix1, iy1, color);
    arcTarget->fillTriangle(ox0, oy0, ix1, iy1, ix0, iy0, color);
    ox0 = ox1; oy0 = oy1; ix0 = ix1; iy0 = iy1;
  }
}

// Arco con bordi sfumati (anti-aliasing) per le fasce delle scale: centro vero del vetro (179.5) e copertura
// per pixel sui bordi, cosi' le curve non fanno gradini e lo spessore resta uguale lungo tutto il giro.
// Angoli in gradi dal punto delle 3 (positivi verso l'alto); mirrorX = lato sinistro (x speculare).
// I pixel interni vanno a righe intere; solo quelli di bordo si calcolano e si mescolano con lo sfondo bg.
static inline uint16_t blend565(uint16_t a, uint16_t b, float t) {   // t = quota di a
  const int ia = (int)(t * 32.0f + 0.5f), ib = 32 - ia;
  const int r = (((a >> 11) & 31) * ia + ((b >> 11) & 31) * ib) >> 5;
  const int g = (((a >> 5) & 63) * ia + ((b >> 5) & 63) * ib) >> 5;
  const int bl = ((a & 31) * ia + (b & 31) * ib) >> 5;
  return (uint16_t)((r << 11) | (g << 5) | bl);
}
static void fillArcAAPart(lgfx::LGFXBase &g, float cx, float cy, float rIn, float rOut, float a0, float a1,
                          bool mirrorX, uint16_t color, uint16_t bg, bool soft0, bool soft1) {
  // a0 < a1, ampiezza <= 90 gradi
  const float c0 = cosf(a0 * DEG_TO_RAD), s0 = sinf(a0 * DEG_TO_RAD);
  const float c1 = cosf(a1 * DEG_TO_RAD), s1 = sinf(a1 * DEG_TO_RAD);
  // riquadro del settore
  float xmin = 1e9f, xmax = -1e9f, ymin = 1e9f, ymax = -1e9f;
  auto addPt = [&](float r, float c, float sn) {
    const float x = cx + (mirrorX ? -1.0f : 1.0f) * r * c, y = cy - r * sn;
    xmin = min(xmin, x); xmax = max(xmax, x); ymin = min(ymin, y); ymax = max(ymax, y);
  };
  addPt(rIn, c0, s0); addPt(rOut, c0, s0); addPt(rIn, c1, s1); addPt(rOut, c1, s1);
  for (int q = -360; q <= 360; q += 90) if ((float)q > a0 && (float)q < a1) addPt(rOut, cosf(q * DEG_TO_RAD), sinf(q * DEG_TO_RAD));
  const int y0 = max(0, (int)floorf(ymin - 1.0f)), y1 = min((int)g.height() - 1, (int)ceilf(ymax + 1.0f));
  const int xa = max(0, (int)floorf(xmin - 1.0f)), xb = min((int)g.width() - 1, (int)ceilf(xmax + 1.0f));
  const float ro2 = (rOut + 0.5f) * (rOut + 0.5f), roIn2 = (rOut - 0.5f) * (rOut - 0.5f);
  const float ri2 = (rIn - 0.5f) > 0.0f ? (rIn - 0.5f) * (rIn - 0.5f) : 0.0f, riIn2 = (rIn + 0.5f) * (rIn + 0.5f);
  g.startWrite();
  for (int y = y0; y <= y1; ++y) {
    const float py = cy - (float)y;
    int runStart = -1;
    for (int x = xa; x <= xb + 1; ++x) {
      float cov = 0.0f;
      if (x <= xb) {
        const float px = (mirrorX ? -1.0f : 1.0f) * ((float)x - cx);
        const float d2 = px * px + py * py;
        if (d2 < ro2 && d2 > ri2) {
          // lati radiali: distanza con segno dalle due rette (positiva dentro)
          const float e0 = c0 * py - s0 * px, e1 = px * s1 - py * c1;
          float ca = 1.0f;
          if (soft0) ca *= constrain(e0 + 0.5f, 0.0f, 1.0f); else if (e0 < 0.0f) ca = 0.0f;
          if (soft1) ca *= constrain(e1 + 0.5f, 0.0f, 1.0f); else if (e1 <= 0.0f) ca = 0.0f;
          if (ca > 0.0f) {
            float cr = 1.0f;
            if (d2 > roIn2 || d2 < riIn2) {
              const float d = sqrtf(d2);
              cr = constrain(rOut + 0.5f - d, 0.0f, 1.0f) * constrain(d - rIn + 0.5f, 0.0f, 1.0f);
            }
            cov = ca * cr;
          }
        }
      }
      if (cov >= 0.97f) { if (runStart < 0) runStart = x; continue; }
      if (runStart >= 0) { g.writeFastHLine(runStart, y, x - runStart, color); runStart = -1; }
      if (cov > 0.03f) g.writePixel(x, y, blend565(color, bg, cov));
    }
  }
  g.endWrite();
}
static void fillArcAA(lgfx::LGFXBase &g, float cx, float cy, float rIn, float rOut, float a0, float a1,
                      bool mirrorX, uint16_t color, uint16_t bg) {
  if (a1 < a0) { const float t = a0; a0 = a1; a1 = t; }
  if (a1 - a0 < 0.01f) return;
  const int parts = max(1, (int)ceilf((a1 - a0) / 90.0f));
  const float da = (a1 - a0) / (float)parts;
  for (int i = 0; i < parts; ++i)   // giunte interne nette (niente riga scura tra i pezzi)
    fillArcAAPart(g, cx, cy, rIn, rOut, a0 + da * i, a0 + da * (i + 1), mirrorX, color, bg, i == 0, i == parts - 1);
}

// Colore di salita e del valore: verde, ma arancione nel tema giallo/nero perche' il verde scuro si legge male sul giallo.
static uint16_t altVarioUpColor() {
  return displayTheme == THEME_YELLOW_BLACK ? lgfx::color565(210, 90, 0) : themeVivid(TFT_GREEN);
}

const int ALT_UNIT_Y = 213;                // riga di "mt/ft" sotto la quota
static void drawAltVarioZeroLine();   // definita piu' sotto
// sign: +1 salita, -1 discesa, 0 nessuna; lit: unita da 1 m/s (mezza tacca).
// Ogni tacca completa vale 2 unita; quella successiva si colora per meta'.
bool altVarioBarsRight = false;   // QFE: barre del vario sul lato destro (ALTIMETER: sinistro)
void drawAltVarioBars(int sign, int lit) {
  const uint16_t ghost = lcd.color565(42, 42, 42);
  const uint16_t upColor = altVarioUpColor();
  const uint16_t downColor = themeVivid(TFT_RED);
  const float pitch = ALT_VARIO_SPAN_DEG / (float)ALT_VARIO_BARS;
  for (int s = 1; s >= -1; s -= 2) {
    for (int i = 0; i < ALT_VARIO_BARS; i++) {
      const float p0 = i * pitch + ALT_VARIO_GAP_DEG * 0.5f;
      const float p1 = (i + 1) * pitch - ALT_VARIO_GAP_DEG * 0.5f;
      const float rIn = (float)ALT_VARIO_R_OUT - ALT_VARIO_T_MIN;
      const int unitsBefore = i * 2;
      const int unitsOn = (sign == s) ? constrain(lit - unitsBefore, 0, 2) : 0;
      const uint16_t color = (unitsOn > 0) ? (s > 0 ? upColor : downColor) : ghost;
      const float mid = (p0 + p1) * 0.5f;
      if (unitsOn >= 2) {
        if (s > 0) fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, p0, p1, color, altVarioBarsRight);
        else fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, -p1, -p0, color, altVarioBarsRight);
      } else if (unitsOn == 1) {
        if (s > 0) fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, p0, mid, color, altVarioBarsRight);
        else fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, -mid, -p0, color, altVarioBarsRight);
        if (s > 0) fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, mid, p1, ghost, altVarioBarsRight);
        else fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, -p1, -mid, ghost, altVarioBarsRight);
      } else {
        if (s > 0) fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, p0, p1, ghost, altVarioBarsRight);
        else fillArcBar(180, 180, rIn, (float)ALT_VARIO_R_OUT, -p1, -p0, ghost, altVarioBarsRight);
      }
    }
  }
  drawAltVarioZeroLine();
}
static void drawAltVarioZeroLine() {
  const int zx = altVarioBarsRight ? 360 - ALT_VARIO_ZERO_X - (ALT_VARIO_ZERO_W + 3) : ALT_VARIO_ZERO_X;
  arcTarget->fillRect(zx, 178, ALT_VARIO_ZERO_W + 3, 3, themeColor(TFT_WHITE));
}

// Aggiorna barre e valore (intero, in verde, con "m/s" piccolo sotto) solo quando cambiano.
// Le tacche e il numero vanno di 1 m/s in 1 m/s (valore arrotondato). Il testo e' disegnato senza
// riquadro di sfondo e il vecchio numero viene cancellato con un rettangolo stretto: non copre altri elementi.
static void updateMainAltVario(bool valid, float vario) {
  int whole = -99998;
  if (valid) whole = (int)lroundf(constrain(vario, -99.0f, 99.0f));
  int sign = 0, lit = 0;
  if (valid && whole != 0) {
    sign = whole > 0 ? 1 : -1;
    lit = abs(whole);
    if (lit > ALT_VARIO_BARS * 2) lit = ALT_VARIO_BARS * 2;
  }
  if (sign != lastMainVarioSign || lit != lastMainVarioLit) {
    drawAltVarioBars(sign, lit);
    lastMainVarioSign = sign;
    lastMainVarioLit = lit;
  }
  if (whole == lastMainVarioTenth) return;
  const bool unitWasShown = (lastMainVarioTenth != 99999 && lastMainVarioTenth != -99998);
  if (lastMainVarioTextW > 0) {
    lcd.fillRect(ALT_VARIO_TEXT_X - 1, 160, lastMainVarioTextW + 2, 36, themeBg());
    lastMainVarioTextW = 0;
  }
  if (!valid) {
    if (unitWasShown) lcd.fillRect(ALT_VARIO_TEXT_X + 4, 194, 50, 24, themeBg());
  } else {
    char buf[12];
    if (whole == 0) snprintf(buf, sizeof(buf), "0");
    else snprintf(buf, sizeof(buf), "%+d", whole);
    const uint16_t green = altVarioUpColor();   // verde (viola nel tema giallo/nero)
    // Carattere semplice (non grassetto), grandezza simile a SET QNH.
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSans18pt7b);
    lcd.setTextSize(1.0f, 1.0f);
    lcd.setTextColor(green);   // sfondo trasparente
    lcd.setTextDatum(ML_DATUM);
    lastMainVarioTextW = lcd.textWidth(buf);
    lcd.drawString(buf, ALT_VARIO_TEXT_X, 178);
    if (!unitWasShown) {
      // "m/s": meta' della grandezza del "mt" della quota (1.3 -> 0.65) e carattere non grassetto per la leggibilita'.
      lcd.setFreeFont(&lgfx::v1::fonts::FreeSans12pt7b);
      lcd.setTextSize((0.65f) * THEME_TEXT_SCALE, (0.65f) * THEME_TEXT_SCALE);
      lcd.setTextColor(green);   // sfondo trasparente
      lcd.setTextDatum(ML_DATUM);
      lcd.drawString("m/s", ALT_VARIO_TEXT_X + 5, 205);
      lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    }
  }
  lastMainVarioTenth = whole;
}

// ---- Tachimetro GS sul lato destro della schermata altimetro ----
// Soglie della fascia colorata (km/h): verde < gsYellowKmh <= giallo < gsRedKmh <= rosso.
// Si regolano con uno swipe sull'arco della pagina GPS SPEED; salvate in NVS ("gs_yel", "gs_red").
int gsYellowKmh = 120;
// Zona lenta dell'arco velocita' (ALTIMETER e GPS SPEED), valori fissi:
// 0 - 65 km/h arancione pieno (sotto 65 lampeggia LOW GS), 65 - 85 km/h arancione a trattini (margine, solo colore).
// Prima del decollo (LOW GS non armato) le due fasce sono grigie, cosi' in rullaggio non si vede arancione.
#define GS_LOW_MARGIN_KMH 85.0f
#define GS_YELLOW_MIN_KMH 90   // la soglia gialla resta sopra la zona lenta
int gsRedKmh = 160;
bool gpsSpeedShowKnots = false;   // ALTM e GPS SPEED: false = km/h, true = nodi ("gs_kt"); tocco sul valore per cambiare
// Fondo scala automatico degli archi GS (ALTIMETER e GPS SPEED), come il QFE: 200 -> 250 -> 300 ... km/h
// (110 -> 135 -> 160 ... kt). Si allarga appena la velocita' supera il fondo scala e torna indietro sotto il
// gradino precedente con il 20% di margine. Le soglie giallo/rosso restano ai km/h impostati.
static const int GS_YELLOW_MAX_KMH = 290, GS_RED_MAX_KMH = 300;   // soglie fino a 300 km/h (sulla scala 0-300)
int gsRangeIdx = 0;                 // 0 = 200 km/h / 110 kt, 1 = 300 km/h / 160 kt (poi 400 e 500 km/h)
// LOW GS: richiamo (non avviso di stallo) quando in volo la velocita' AL SUOLO scende sotto 65 km/h (35 kt).
// Si arma solo dopo il decollo, appena la GS raggiunge 100 km/h; si disarma all'atterraggio.
// La GS non e' la velocita' rispetto all'aria: con vento in coda si puo' essere lenti anche con GS alta.
const float LOW_GS_KMH = 65.0f, LOW_GS_ARM_KMH = 90.0f;
bool lowGsArmed = false, lowGsNow = false;
// Rettangolo arancione 65-85 km/h: lampeggia quando, dopo aver superato 90 km/h, la velocita' rientra tra 65 e 85.
// Si disarma solo sotto 65 km/h (va superato di nuovo 90 per riarmarlo).
bool gsRectArmed = false, gsRectBlinkNow = false;
void updateLowGsState() {
  float sp = 0.0f; bool ok = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)2) == pdTRUE) {
    sp = gps_ground_speed_kmh; ok = gpsDataValid && isfinite(sp);
    xSemaphoreGive(sensorDataMutex);
  } else {
    return;
  }
  if (!automaticFlightRunning) lowGsArmed = false;
  else if (ok && sp >= LOW_GS_ARM_KMH - 0.5f) lowGsArmed = true;   // si arma in volo a 90 km/h (89.6 appare gia' "90")
  lowGsNow = lowGsArmed && ok && sp < LOW_GS_KMH;   // LOW GS resta attivo anche vicino alla meta (finale)
  if (ok && sp >= LOW_GS_ARM_KMH - 0.5f) gsRectArmed = true;
  else if (ok && sp < LOW_GS_KMH) gsRectArmed = false;
  gsRectBlinkNow = gsRectArmed && ok && sp >= LOW_GS_KMH && sp <= GS_LOW_MARGIN_KMH;
}
static bool lowGsBlinkOn() { return (millis() / 500UL) % 2UL == 0UL; }   // lampeggio della scritta LOW GS
static const uint16_t LOW_GS_COLOR = 0xFCC0;   // arancione (255,154,0)
// Fondo scala: 0-200 km/h, superati i 200 direttamente 0-300 (in nodi 110 -> 160); oltre 400 e 500 km/h.
// Con GS 150 si parte da 0-150 km/h (0-80 kt), poi 200, 300, 400. La 0-150 resta solo se il rosso
// parte da 150 o meno (e il giallo sotto 150): altrimenti 0-200, come gia' succede tra 200 e 300.
static float gsScaleMaxFor(bool kt, int idx) {
  static const float KMH[4] = { 200.0f, 300.0f, 400.0f, 500.0f }, KT[4] = { 110.0f, 160.0f, 215.0f, 270.0f };
  static const float KMH150[4] = { 150.0f, 200.0f, 300.0f, 400.0f }, KT150[4] = { 80.0f, 110.0f, 160.0f, 215.0f };
  idx = constrain(idx, 0, 3);
  if (gsScale150) return kt ? KT150[idx] : KMH150[idx];
  return kt ? KT[idx] : KMH[idx];
}
static bool gsScaleIs150(bool kt, float maxU) { return !kt && maxU <= 150.5f; }   // scala 0-150 km/h: numeri ogni 25, tacche ogni 5
static bool gsUpdateAutoRange(float speedKmh, bool valid) {
  if (!valid || !isfinite(speedKmh)) speedKmh = 0.0f;   // anche senza GPS la scala segue le soglie
  const bool kt = gpsSpeedShowKnots;
  const float u = kt ? speedKmh / 1.852f : speedKmh;
  const float hyst = kt ? 5.0f : 10.0f;   // si torna alla scala piu' piccola solo 10 km/h (5 kt) sotto il suo fondo
  // Soglie: con la linea gialla a fondo scala (200 km/h, 110 kt) o la rossa oltre, scala 0-300 km/h (0-160 kt)
  const float uY = kt ? gsYellowKmh / 1.852f : (float)gsYellowKmh, uR = kt ? gsRedKmh / 1.852f : (float)gsRedKmh;
  int minIdx = 0;
  if (uY >= gsScaleMaxFor(kt, 0) - 0.5f) minIdx = 1;
  while (minIdx < 3 && uR > gsScaleMaxFor(kt, minIdx) + 0.5f) minIdx++;
  int idx = constrain(gsRangeIdx, 0, 3);
  while (idx < 3 && u > gsScaleMaxFor(kt, idx)) idx++;
  while (idx > minIdx && u < gsScaleMaxFor(kt, idx - 1) - hyst) idx--;
  if (idx < minIdx) idx = minIdx;
  if (idx == gsRangeIdx) return false;
  gsRangeIdx = idx;
  return true;
}
// Passo dei numeri e delle tacche: piu' larghi quando il fondo scala cresce, per non affollare l'arco.
static int gsLabelStep(bool kt, float maxU) { return gsScaleIs150(kt, maxU) ? 25 : (kt ? (maxU <= 135.0f ? 25 : 50) : (maxU <= 250.0f ? 50 : 100)); }
static int gsTickStep(bool kt, float maxU) { return gsScaleIs150(kt, maxU) ? 5 : (kt ? (maxU <= 135.0f ? 5 : 10) : (maxU <= 250.0f ? 10 : 20)); }
// Zona della lettura GS (sotto "mt", sopra SET QNH)
const int GS_LABEL_Y = 243;
const int GS_VALUE_Y = 268;
// Zona pulita ad ogni aggiornamento: stretta sul testo, cosi' non tocca gli archi del vario e della GS.
const int GS_READOUT_X0 = 105, GS_READOUT_X1 = 255, GS_READOUT_Y0 = 234, GS_READOUT_Y1 = 284;
// SET QNH piccolo, nella parte piu' bassa del display
const int SETQNH_BOX_X = 125, SETQNH_BOX_Y = 300, SETQNH_BOX_W = 110, SETQNH_BOX_H = 32;
static int gsLastShownKmh = -2;          // -1 = GPS non valido, -2 = da ridisegnare
static bool gsPointerDrawn = false;
static int gsPtrPts[6] = {0, 0, 0, 0, 0, 0};

static float gsUnitMax() { return gsScaleMaxFor(gpsSpeedShowKnots, gsRangeIdx); }        // fondo scala (automatico)
static float gsKmhToUnit(float kmh) { return gpsSpeedShowKnots ? kmh / 1.852f : kmh; }
// Angolo sull'arco destro per un valore nell'unita' mostrata (km/h o nodi): 0 in basso (-60), fondo scala in alto (+60).
static float gsGaugeAngle(float unitVal) {
  return -60.0f + 120.0f * constrain(unitVal, 0.0f, gsUnitMax()) / gsUnitMax();
}
static int altGsDragTarget = 0;              // 0 nessuna, 1 soglia gialla, 2 soglia rossa (swipe sull'arco ALTM)
static bool gsScaleNeedsErase = false;       // cambio unita': cancella tacche e scritte vecchie
// Punto a raggio r e angolo aDeg (dal punto delle 3, positivo verso l'alto), spostato di off px in perpendicolare.
static void gsPolar(float r, float aDeg, float off, int &x, int &y) {
  const float a = aDeg * DEG_TO_RAD;
  x = (int)lroundf(180.0f + r * cosf(a) - off * sinf(a));
  y = (int)lroundf(180.0f - r * sinf(a) - off * cosf(a));
}
// Mescola due colori 565: a = 0 -> c0, a = 1 -> c1
static uint16_t gsMix565(uint16_t c0, uint16_t c1, float a) {
  a = constrain(a, 0.0f, 1.0f);
  const int r0 = (c0 >> 11) & 31, g0 = (c0 >> 5) & 63, b0 = c0 & 31, r1 = (c1 >> 11) & 31, g1 = (c1 >> 5) & 63, b1 = c1 & 31;
  const int r = r0 + (int)lroundf((r1 - r0) * a), gg = g0 + (int)lroundf((g1 - g0) * a), b = b0 + (int)lroundf((b1 - b0) * a);
  return (uint16_t)((r << 11) | (gg << 5) | b);
}
// Lancetta "opzione 2, scia da 0": l'arco si riempie di luce bianca dallo 0 alla lancetta, sempre piu' forte
// verso la lancetta e verso il bordo esterno; lancetta arancione-rossa, piena verso il bordo e tenue verso il centro. Il fondo e' a tinta unita: la trasparenza e' un colore mescolato con il fondo.
// arcSel: 0 = GPS SPEED (gpsArcPt), 1 = arco GS dell'ALTIMETER (gsPolar), 2 = arco QFE a sinistra (gsPolar specchiato).
// aZero = angolo dello 0 della scala, aTip = angolo della lancetta; (ox, oy) = origine dello sprite.
static void arcPolarSel(int arcSel, float r, float deg, int &x, int &y);
// Tabelle calcolate una volta all'accensione per la scia: per ogni blocco di 2x2 pixel del display l'angolo
// (centesimi di grado) e la distanza dal centro (ottavi di pixel). Nella PSRAM (circa 130 KB); senza memoria
// la scia si calcola come prima, punto per punto.
static int16_t *sweepLutDeg = nullptr;
static uint16_t *sweepLutR8 = nullptr;
void sweepLutInit() {
  if (sweepLutDeg != nullptr) return;
  int16_t *d = (int16_t *)heap_caps_malloc(180 * 180 * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  uint16_t *r = (uint16_t *)heap_caps_malloc(180 * 180 * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (d == nullptr || r == nullptr) { if (d) heap_caps_free(d); if (r) heap_caps_free(r); return; }
  for (int by = 0; by < 180; ++by) {
    const float dy = 180.0f - (by * 2.0f + 0.5f);
    for (int bx = 0; bx < 180; ++bx) {
      const float dx = bx * 2.0f + 0.5f - 180.0f;
      d[by * 180 + bx] = (int16_t)lroundf(atan2f(dy, dx) * 5729.578f);
      r[by * 180 + bx] = (uint16_t)lroundf(sqrtf(dx * dx + dy * dy) * 8.0f);
    }
  }
  sweepLutDeg = d; sweepLutR8 = r;
}
static void arcSweepC(lgfx::LGFXBase &t, int arcSel, int ox, int oy, float aZero, float aTip, float rIn, float rOut, uint16_t bg) {
  const int lum = (((bg >> 11) & 31) * 2 + ((bg >> 5) & 63) + (bg & 31) * 2);
  const bool darkBg = lum < 125;
  const uint16_t veil = darkBg ? lgfx::color565(0, 255, 255) : lgfx::color565(0, 110, 130);   // scia ciano forte (si vede al sole)
  (void)darkBg;
  const float TRAIL = 150.0f, AMAX = 1.0f;    // ciano pieno vicino alla lancetta, sfumato del tutto 150 gradi dietro
  const float dir = aZero > aTip ? 1.0f : -1.0f;               // verso dello 0, cioe' "dietro" la lancetta
  const float span = min(TRAIL, fabsf(aZero - aTip));   // oltre TRAIL la scia e' gia' spenta
  // Scia con sfumatura continua (niente archi a gradini): l'intensita' si calcola a blocchi di 2x2 pixel
  // (4 volte meno calcoli), il colore di ogni pixel con dithering ordinato 4x4 per una sfumatura liscia.
  if (span > 0.3f) {
    static const uint8_t BAYER[16] = { 0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5 };
    const int r0b = (bg >> 11) & 31, g0b = (bg >> 5) & 63, b0b = bg & 31;
    const float dR = (float)(((veil >> 11) & 31) - r0b), dG = (float)(((veil >> 5) & 63) - g0b), dB = (float)((veil & 31) - b0b);
    // La scia arriva fino al bordo dell'arco colorato: i pixel vicino al bordo esterno si controllano uno per uno
    // (prima il controllo a blocchi di 2x2 lasciava qualche pixel nero tra la scia e la fascia colorata).
    const float rEdge = rOut + 1.0f, rEdge2 = rEdge * rEdge;   // 1 pixel oltre rOut: tocca la fascia colorata
    const float rB = rOut + 3.0f;                                   // area esaminata: un po' oltre il bordo
    const float rIn2 = rIn * rIn, rOut2 = rB * rB, invSpanR = 1.0f / (rOut - rIn);
    const int W = t.width(), H = t.height();
    int yS = max(0, (int)floorf(180.0f - rB) - oy), yE = min(H - 1, (int)ceilf(180.0f + rB) - oy);
    yS &= ~1;
    for (int y = yS; y <= yE; y += 2) {
      const float dy = 180.0f - ((float)(y + oy) + 0.5f);   // centro del blocco
      const float dy2 = dy * dy;
      if (dy2 > rOut2) continue;
      const float xo = sqrtf(rOut2 - dy2);
      const float xi = dy2 < rIn2 ? sqrtf(rIn2 - dy2) : -1.0f;
      for (int side = 0; side < 2; ++side) {   // sinistra e destra del foro centrale
        int xA, xB;
        if (xi < 0.0f) { if (side) break; xA = (int)floorf(180.0f - xo); xB = (int)ceilf(180.0f + xo); }
        else if (side == 0) { xA = (int)floorf(180.0f - xo); xB = (int)ceilf(180.0f - xi); }
        else { xA = (int)floorf(180.0f + xi); xB = (int)ceilf(180.0f + xo); }
        xA = max(xA - ox, 0) & ~1; xB = min(xB - ox, W - 1);
        for (int x = xA; x <= xB; x += 2) {
          float dx = (float)(x + ox) + 0.5f - 180.0f;
          float rr0, deg;
          const int sx = x + ox, sy = y + oy;
          if (sweepLutDeg != nullptr && sx >= 0 && sx < 360 && sy >= 0 && sy < 360) {   // tabelle dell'accensione
            const int li = (sy >> 1) * 180 + (sx >> 1);
            rr0 = sweepLutR8[li] * 0.125f;
            deg = sweepLutDeg[li] * 0.01f;
          } else {
            rr0 = sqrtf(dx * dx + dy2);
            deg = atan2f(dy, dx) * 57.29578f;
          }
          if (rr0 < rIn || rr0 > rOut + 2.5f) continue;
          const bool nearEdge = rr0 > rOut - 1.5f;
          if (arcSel == 2) deg = deg >= 0.0f ? 180.0f - deg : -180.0f - deg;   // QFE: arco specchiato
          if (arcSel == 0 && deg < -135.0f) deg += 360.0f; // GPS SPEED: scala da 225 a -45 gradi
          const float back = (deg - aTip) * dir;            // gradi dietro la lancetta, verso lo 0
          if (back < 0.0f || back > span) continue;
          float fa = 1.0f - back / TRAIL; fa *= fa;
          const float q = min(1.0f, (rr0 - rIn) * invSpanR);
          const float al = AMAX * fa * (0.45f + 0.55f * q);   // piu' forte verso la scala, ma piena gia' all'interno
          const float vr = r0b + dR * al, vg = g0b + dG * al, vb = b0b + dB * al;
          for (int py = 0; py < 2; ++py) for (int px = 0; px < 2; ++px) {
            const int X = x + px, Y = y + py;
            if (X >= W || Y > yE) continue;
            if (nearEdge) {   // bordo esterno: solo i pixel il cui centro e' dentro l'arco
              const float ex = (float)(X + ox) + 0.5f - 180.0f, ey = 180.0f - ((float)(Y + oy) + 0.5f);
              if (ex * ex + ey * ey > rEdge2) continue;
            }
            const float d = (BAYER[((Y & 3) << 2) | (X & 3)] + 0.5f) / 16.0f;
            const int rr = min(31, (int)(vr + d)), gg = min(63, (int)(vg + d)), bb = min(31, (int)(vb + d));
            t.drawPixel(X, Y, (uint16_t)((rr << 11) | (gg << 5) | bb));
          }
        }
      }
    }
  }
  // arco fisso interno: linea grigia chiara di circa 1,6 pixel sul bordo interno della scia, parallela alla scala
  {
    const float aFull = arcSel == 0 ? -45.0f : 60.0f;   // fondo scala
    const float rA = rIn - 1.0f;
    const uint16_t ga = lgfx::color565(154, 154, 154);
    const uint16_t gb = gsMix565(bg, ga, 0.6f);          // seconda riga piu' tenue: spessore circa 1,6 px
    const float a0 = min(aZero, aFull), a1 = max(aZero, aFull);
    for (int ring = 0; ring < 2; ++ring) {
      const float rr = rA - ring;
      const float stepA = 0.5f / rr * 57.29578f;         // mezzo pixel per passo: linea continua
      int lx = -9999, ly = -9999;
      for (float a = a0; a <= a1; a += stepA) {
        int px, py;
        arcPolarSel(arcSel, rr, a, px, py);
        if (px == lx && py == ly) continue;
        lx = px; ly = py;
        t.drawPixel(px - ox, py - oy, ring == 0 ? ga : gb);
      }
    }
  }
  // (la lancetta si disegna a parte con arcNeedleC, alla fine della pagina: sopra tacche, numeri e scritte)
}
// Lancetta arancione-rossa della scia (opzione 2): si disegna per ultima, cosi' resta sopra fasce, tacche e numeri.
// Stessi parametri di arcSweepC; i colori verso il centro si mescolano con la scia come prima.
static void arcNeedleC(lgfx::LGFXBase &t, int arcSel, int ox, int oy, float aTip, float rIn, float rOut, uint16_t bg) {
  (void)bg;
  // Lancetta bianca piena (senza sfumatura) con bordino nero, cosi' si stacca anche sopra la fascia rossa.
  // Da 8 px dentro la scia fino a r 172 (3 px prima dell'arco bianco 0-65, che va da 175 a 179), sempre piu' larga;
  // spessore in proporzione alla lunghezza (rispetto alla vecchia punta a rOut + 3).
  const float n0 = rIn - 8.0f, n1 = ARC_R_OUT - 4.0f - 3.0f, wk = (n1 - n0) / (rOut + 3.0f - n0);   // punta a 3 px dall'arco bianco (175)
  const int NS = 12;
  for (int pass = 0; pass < 2; ++pass) {   // 0 = bordino nero, 1 = rosso pieno sopra
    for (int k = 0; k < NS; ++k) {
      const float tt = (float)(k + 1) / NS;
      const float ra = n0 + (n1 - n0) * k / NS, rb = n0 + (n1 - n0) * (k + 1) / NS;
      const float w = (0.8f + 2.35f * tt) * wk;
      int xa, ya, xb, yb;
      arcPolarSel(arcSel, ra, aTip, xa, ya); arcPolarSel(arcSel, rb, aTip, xb, yb);
      if (pass == 0) t.drawWideLine(xa - ox, ya - oy, xb - ox, yb - oy, w + 1.3f, (uint16_t)TFT_BLACK);
      else t.drawWideLine(xa - ox, ya - oy, xb - ox, yb - oy, w, themeColor(TFT_WHITE));   // bianca (nera sui temi chiari)
    }
  }
}

// ALTIMETER: scia al posto della lancetta sull'arco destro (come GPS SPEED). Da 0 (in basso, -60 gradi)
// alla velocita': velo bianco sempre piu' luminoso verso la punta (grigio scuro sui temi chiari) e linea arancione.
// (ox, oy) = origine dello sprite sul display.
static void gsDrawSpeedSweepOn(lgfx::LGFXBase &t, int ox, int oy, float aEnd, float rIn, float rOut, uint16_t bg, uint16_t tipColor) {
  (void)tipColor;
  arcSweepC(t, 1, ox, oy, -60.0f, constrain(aEnd, -60.0f, 60.0f), rIn, rOut, bg);   // proposta C
}
// Colore della fascia "normale" degli archi (ALTIMETER, QFE, GPS SPEED): blu al posto del verde,
// perche' al sole verde e giallo si confondono; blu e giallo restano sempre distinguibili.
static uint16_t arcSafeColor() { return themeVivid(TFT_GREEN); }   // fascia normale degli archi: sempre verde (BLUE ARCS tolto)
// Zona lenta: 0-65 km/h senza fascia (tacche della scala allungate fino al bordo, piu' tacche intermedie);
// rettangolo 65-85 km/h sempre arancione, lampeggiante con gsRectBlinkNow.
static bool gsRectVisible() { return !gsRectBlinkNow || lowGsBlinkOn(); }
static uint16_t gsSpeedColor(float kmh) {
  if (lowGsArmed && kmh < GS_LOW_MARGIN_KMH) return LOW_GS_COLOR;   // zona lenta (piena o a trattini)
  if (kmh < (float)gsYellowKmh) return arcSafeColor();
  if (kmh < (float)gsRedKmh) return themeVivid(TFT_YELLOW);
  return themeVivid(TFT_RED);
}
static bool gsLabelsDrawnKt = false;      // unita' e fondo scala dei numeri attualmente sul display
static float gsLabelsDrawnMax = 0.0f;
// Disegna la scala GS (fascia, trattini, tacche, numeri) su una superficie qualsiasi, spostata di (ox, oy).
static void drawGsGaugeScaleOn(lgfx::LGFXBase &t, int ox, int oy) {
  lgfx::LGFXBase *prevArc = arcTarget;
  arcTarget = &t;
  const int ccx = 180 - ox, ccy = 180 - oy;
  const float maxU = gsUnitMax();
  const float lowU = constrain(gsKmhToUnit(LOW_GS_KMH), 0.0f, maxU);
  const float marU = constrain(gsKmhToUnit(GS_LOW_MARGIN_KMH), lowU, maxU);
  const float yel = constrain(gsKmhToUnit((float)gsYellowKmh), marU, maxU);
  const float red = constrain(gsKmhToUnit((float)gsRedKmh), yel, maxU);
  const uint16_t lowColor = LOW_GS_COLOR;
  // Fasce con bordi sfumati, centro 179.5, bordo esterno 177.5 (filo nero costante dal vetro)
  const float acx = 179.5f - (float)ox, acy = 179.5f - (float)oy, RO = ARC_R_OUT;
  const uint16_t abg = themeBg();
  if (lowU > 0.0f) fillArcAA(t, acx, acy, RO - 4.0f, RO, gsGaugeAngle(0), gsGaugeAngle(lowU), false, themeColor(TFT_WHITE), abg);   // 0-65: arco bianco esterno 4 px
  if (marU > lowU && gsRectVisible()) {   // 65 - 85 km/h: solo contorno (2.5 px), staccato di ~3 px dai separatori neri
    const float a0 = gsGaugeAngle(lowU), a1 = gsGaugeAngle(marU);
    const float IN = 1.0f, CAP = 0.85f;   // gradi (a r 170: ~3 px e ~2.5 px)
    if (a1 - a0 > 2.0f * IN + 2.0f * CAP) {
      fillArcAA(t, acx, acy, 162.0f, 164.5f, a0 + IN, a1 - IN, false, lowColor, abg);
      fillArcAA(t, acx, acy, RO - 2.5f, RO, a0 + IN, a1 - IN, false, lowColor, abg);
      fillArcAA(t, acx, acy, 162.0f, RO, a0 + IN, a0 + IN + CAP, false, lowColor, abg);
      fillArcAA(t, acx, acy, 162.0f, RO, a1 - IN - CAP, a1 - IN, false, lowColor, abg);
    } else fillArcAA(t, acx, acy, 162.0f, RO, a0, a1, false, lowColor, abg);
  }
  if (yel > marU) fillArcAA(t, acx, acy, 162.0f, RO, gsGaugeAngle(marU), gsGaugeAngle(yel), false, arcSafeColor(), abg);
  if (red > yel) fillArcAA(t, acx, acy, 162.0f, RO, gsGaugeAngle(yel), gsGaugeAngle(red), false, themeVivid(TFT_YELLOW), abg);
  if (red < maxU) fillArcAA(t, acx, acy, 162.0f, RO, gsGaugeAngle(red), gsGaugeAngle(maxU), false, themeVivid(TFT_RED), abg);
  // Trattini neri (4 px) ai bordi delle fasce: zona lenta, margine, giallo e rosso
  for (int e = 0; e < 4; ++e) {
    const float ev = e == 0 ? lowU : (e == 1 ? marU : (e == 2 ? yel : red));
    if (ev <= 0.0f || ev >= maxU) continue;
    int x0, y0, x1, y1;
    gsPolar(161.0f, gsGaugeAngle(ev), 0.0f, x0, y0);
    gsPolar(180.0f, gsGaugeAngle(ev), 0.0f, x1, y1);
    drawThickLineOn(t, x0 - ox, y0 - oy, x1 - ox, y1 - oy, TFT_BLACK, 4);   // trattino nero 4 px
  }
  const uint16_t tickColor = themeColor(TFT_WHITE);
  const int tickStep = gsTickStep(gpsSpeedShowKnots, maxU);
  const int majorStep = gsLabelStep(gpsSpeedShowKnots, maxU);
  for (int k = 0; k <= (int)maxU; k += tickStep) {
    int x0, y0, x1, y1;
    gsPolar(k % majorStep == 0 ? 142.0f : 151.0f, gsGaugeAngle((float)k), 0.0f, x0, y0);
    gsPolar((float)k < lowU - 0.01f ? ARC_TICK_R : 161.0f, gsGaugeAngle((float)k), 0.0f, x1, y1);   // 0-65: fino all'arco bianco
    drawThickLineOn(t, x0 - ox, y0 - oy, x1 - ox, y1 - oy, tickColor, 2);
  }
  for (float k = tickStep / 2.0f; !gsScaleIs150(gpsSpeedShowKnots, maxU) && k < lowU - 0.01f; k += tickStep) {   // 0-65: tacche intermedie corte (5, 15, 25...); in 0-150 le tacche sono gia' ogni 5
    int x0, y0, x1, y1;
    gsPolar(167.0f, gsGaugeAngle(k), 0.0f, x0, y0);
    gsPolar(ARC_TICK_R, gsGaugeAngle(k), 0.0f, x1, y1);
    t.drawWideLine(x0 - ox, y0 - oy, x1 - ox, y1 - oy, 0.8f, tickColor);
  }
  t.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  t.setTextSize(1.1f * THEME_TEXT_SCALE, 1.1f * THEME_TEXT_SCALE);   // numeri dell'arco +30%
  t.setTextDatum(MC_DATUM);
  t.setTextColor(themeColor(TFT_LIGHTGREY));   // sfondo trasparente
  for (int v = majorStep; v <= (int)maxU; v += majorStep) {
    int lx, ly;
    const float la = gsGaugeAngle((float)v);
    gsPolar(fabsf(la) < 15.0f ? 120.0f : 128.0f, la, 0.0f, lx, ly);
    char b[6];
    snprintf(b, sizeof(b), "%d", v);
    t.drawString(b, lx - ox, ly - oy);
  }
  t.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  gsLabelsDrawnKt = gpsSpeedShowKnots;   // per cancellarli bene al prossimo cambio di unita' o di scala
  gsLabelsDrawnMax = maxU;
  arcTarget = prevArc;
}
// Cancella i numeri della scala dell'unita' precedente ridisegnandoli nel colore dello sfondo:
// il "200" in alto esce dal settore cancellato con fillArcBar e restava visibile in parte.
static void eraseGsLabels() {
  const bool knots = gsLabelsDrawnKt;
  const float maxU = gsLabelsDrawnMax;
  if (maxU <= 0.0f) return;
  const int step = gsLabelStep(knots, maxU);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1.1f * THEME_TEXT_SCALE, 1.1f * THEME_TEXT_SCALE);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(themeBg());
  for (int v = step; v <= (int)maxU; v += step) {
    const float la = -60.0f + 120.0f * (float)v / maxU;
    int lx, ly;
    gsPolar(fabsf(la) < 15.0f ? 120.0f : 128.0f, la, 0.0f, lx, ly);
    char b[6];
    snprintf(b, sizeof(b), "%d", v);
    for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) lcd.drawString(b, lx + dx, ly + dy);   // con margine
  }
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
}
static void drawGsGaugeScale() {
  if (gsScaleNeedsErase) {
    eraseGsLabels();   // numeri della vecchia unita' / del vecchio fondo scala
    // settore destro da r=105 al bordo: tacche, scritte e fascia vecchie
    fillArcBar(180, 180, 105.0f, 179.0f, -64.0f, 64.0f, themeBg(), true);
    gsScaleNeedsErase = false;
  }
  drawGsGaugeScaleOn(lcd, 0, 0);
}
// Arco GS composto in uno sprite e spinto in un colpo solo: niente vibrazione di fascia e lancetta.
// I pixel di colore GS_SPR_KEY non vengono scritti (resta cio' che c'e' sul display, es. la quota).
static LGFX_Sprite gsGaugeSprite(&lcd);
static bool gsGaugeSpriteTried = false, gsGaugeSpriteOk = false;
static const int GS_SPR_X = 218, GS_SPR_Y = 14, GS_SPR_W = 142, GS_SPR_H = 332;
static const uint16_t GS_SPR_KEY = 0xF81F;   // magenta: colore "trasparente"
static bool createSmallSprite(LGFX_Sprite &sp, int w, int h);
// Lancetta che scorre: il valore disegnato insegue quello vero (costante di tempo 120 ms) invece di saltare
// tra un dato GPS e il successivo. Il numero mostrato resta quello vero.
static float needleFollow(float &shown, unsigned long &lastMs, float target, bool valid) {
  const unsigned long now = millis();
  const float dt = (float)(now - lastMs);
  lastMs = now;
  if (!valid || !isfinite(target)) { shown = 0.0f; return shown; }
  if (dt > 1000.0f || fabsf(target - shown) < 0.05f) shown = target;   // pagina appena aperta: subito al valore
  else shown += (target - shown) * (1.0f - expf(-dt / 120.0f));
  return shown;
}
static bool drawGsGaugeComposite(bool valid, float unitVal) {
  if (!gsGaugeSpriteTried) {
    gsGaugeSpriteTried = true;
    gsGaugeSpriteOk = createSmallSprite(gsGaugeSprite, GS_SPR_W, GS_SPR_H);
    if (!gsGaugeSpriteOk) AVEA_LOGLN("[ALT] sprite arco GS non allocato: disegno diretto");
  }
  if (!gsGaugeSpriteOk) return false;
  if (gsScaleNeedsErase) {   // cambio unita': via anche i numeri vecchi (fuori dalla parte opaca dello sprite)
    eraseGsLabels();
    fillArcBar(180, 180, 105.0f, 179.0f, -64.0f, 64.0f, themeBg(), true);
    gsScaleNeedsErase = false;
  }
  LGFX_Sprite &sp = gsGaugeSprite;
  sp.fillSprite(GS_SPR_KEY);
  lgfx::LGFXBase *prevArc = arcTarget;
  arcTarget = &sp;
  // parte opaca: dal raggio 120 al bordo (cancella la lancetta vecchia, anche il pezzo verso il centro e il bagliore)
  fillArcBar(180 - GS_SPR_X, 180 - GS_SPR_Y, 120.0f, 180.0f, -64.0f, 64.0f, themeBg(), true);
  arcTarget = prevArc;
  // scia della velocita' sotto tacche e numeri (dentro la parte opaca: si rinnova a ogni disegno)
  if (valid) gsDrawSpeedSweepOn(sp, GS_SPR_X, GS_SPR_Y, gsGaugeAngle(unitVal), 131.0f, 161.0f, themeBg(), themeVivid(TFT_ORANGE));
  drawGsGaugeScaleOn(sp, GS_SPR_X, GS_SPR_Y);
  if (valid) arcNeedleC(sp, 1, GS_SPR_X, GS_SPR_Y, constrain(gsGaugeAngle(unitVal), -60.0f, 60.0f), 131.0f, 161.0f, themeBg());   // lancetta in primo piano
  sp.pushSprite(GS_SPR_X, GS_SPR_Y, GS_SPR_KEY); drawPageLockOverlay();
  return true;
}
// Aggiorna lancetta e lettura solo quando cambia la velocita' (km/h interi); force = ridisegno completo.
static void updateMainGsGauge(bool force) {
  float speed = 0.0f;
  bool valid = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
    speed = gps_ground_speed_kmh;
    valid = gpsDataValid && isfinite(speed);
    xSemaphoreGive(sensorDataMutex);
  } else if (!force) {
    return;
  }
  if (gsUpdateAutoRange(speed, valid)) {   // fondo scala cambiato: ridisegno completo dell'arco
    gsScaleNeedsErase = true;
    force = true;
    lastPrintedAlt = -99999;   // il settore cancellato puo' toccare le ultime cifre della quota
  }
  const int kmh = valid ? (int)lroundf(constrain(gsKmhToUnit(speed), 0.0f, 999.0f)) : -1;   // nell'unita' mostrata
  // lancetta che scorre verso la velocita' vera
  static float ndlKmh = 0.0f;
  static unsigned long ndlMs = 0, ndlDrawMs = 0;
  static int ndlLastKey = -2;
  needleFollow(ndlKmh, ndlMs, speed, valid);
  const float ndlUnit = constrain(gsKmhToUnit(ndlKmh), 0.0f, 999.0f);
  const int ndlKey = valid ? (int)lroundf(ndlUnit * 4.0f) : -1;   // quarti di unita'
  const uint8_t lowKey = (lowGsNow ? (lowGsBlinkOn() ? 2 : 1) : 0) |
                         (gsRectBlinkNow ? (lowGsBlinkOn() ? 8 : 4) : 0);   // LOW GS e rettangolo 65-85 lampeggianti
  static uint8_t lastLowKey = 0;
  static bool lastArmed = false;
  if (lowGsArmed != lastArmed) { lastArmed = lowGsArmed; force = true; }   // zona lenta grigia <-> arancione
  if (!force && kmh == gsLastShownKmh && lowKey == lastLowKey) {
    // numero invariato: si muove solo la lancetta (al massimo 40 volte al secondo)
    if (ndlKey != ndlLastKey && millis() - ndlDrawMs >= 25UL && drawGsGaugeComposite(valid, ndlUnit)) {
      ndlLastKey = ndlKey; ndlDrawMs = millis();
    }
    return;
  }
  ndlLastKey = ndlKey; ndlDrawMs = millis();
  const bool gaugeChanged = force || kmh != gsLastShownKmh;
  lastLowKey = lowKey;
  gsLastShownKmh = kmh;
  const char *gsUnit = gpsSpeedShowKnots ? "kt" : "km/h";
  const uint16_t bg = themeBg();
  if (!gaugeChanged) {
    // solo lampeggio LOW GS: arco e lancetta restano come sono
  } else if (drawGsGaugeComposite(valid, ndlUnit)) {
    gsPointerDrawn = false;   // arco e lancetta gia' disegnati dallo sprite
  } else {
  if (gsPointerDrawn) {
    lcd.fillTriangle(gsPtrPts[0], gsPtrPts[1], gsPtrPts[2], gsPtrPts[3], gsPtrPts[4], gsPtrPts[5], bg);
    lcd.drawTriangle(gsPtrPts[0], gsPtrPts[1], gsPtrPts[2], gsPtrPts[3], gsPtrPts[4], gsPtrPts[5], bg);   // anche il contorno
    gsPointerDrawn = false;
  }
  drawGsGaugeScale();
  if (valid) {
    const float a = gsGaugeAngle((float)kmh);
    gsPolar(172.0f, a, 0.0f, gsPtrPts[0], gsPtrPts[1]);    // un po' piu' grande
    gsPolar(136.0f, a, 10.0f, gsPtrPts[2], gsPtrPts[3]);
    gsPolar(136.0f, a, -10.0f, gsPtrPts[4], gsPtrPts[5]);
    drawArcPointerTri(lcd, gsPtrPts[0], gsPtrPts[1], gsPtrPts[2], gsPtrPts[3], gsPtrPts[4], gsPtrPts[5]);
    gsPointerDrawn = true;
  }
  }   // fine disegno diretto (senza sprite)
  // Lettura numerica nello spazio tra la quota e SET QNH: "GS" piccolo e sotto "112 km/h"
  // con lo stesso carattere e la stessa grandezza del valore del variometro (FreeSans18pt).
  lcd.fillRect(GS_READOUT_X0, GS_READOUT_Y0, GS_READOUT_X1 - GS_READOUT_X0, GS_READOUT_Y1 - GS_READOUT_Y0, bg);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1.0f * THEME_TEXT_SCALE, 1.0f * THEME_TEXT_SCALE);
  lcd.setTextDatum(MC_DATUM);
  if (lowGsNow) {   // LOW GS arancione lampeggiante al posto di GS
    lcd.setTextColor(LOW_GS_COLOR);
    if (lowKey == 2) lcd.drawString("LOW GS", 180, GS_LABEL_Y);
  } else {
    lcd.setTextColor(themeColor(TFT_CYAN));
    lcd.drawString("GS", 180, GS_LABEL_Y);
  }
  char buf[20];
  // Tema a colori: colore della fascia; altri temi: colore del testo del tema
  uint16_t valColor = !valid ? themeColor(TFT_DARKGREY)
                    : (displayTheme == THEME_COLOR ? gsSpeedColor(speed) : themeColor(TFT_WHITE));
  if (altGsDragTarget != 0) {
    // swipe sull'arco in corso: mostra la soglia che si sta spostando, nel suo colore
    const int b = (int)lroundf(gsKmhToUnit((float)(altGsDragTarget == 1 ? gsYellowKmh : gsRedKmh)));
    snprintf(buf, sizeof(buf), altGsDragTarget == 1 ? "YEL %d" : "RED %d", b);
    valColor = themeVivid(altGsDragTarget == 1 ? TFT_YELLOW : TFT_RED);
  } else if (valid) snprintf(buf, sizeof(buf), "%d %s", kmh, gsUnit);
  else snprintf(buf, sizeof(buf), "-- %s", gsUnit);
  if (lowGsNow && altGsDragTarget == 0) valColor = LOW_GS_COLOR;   // valore arancione con LOW GS (km/h o nodi)
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSans18pt7b);
  lcd.setTextSize(1.0f, 1.0f);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(valColor);
  lcd.drawString(buf, 180, GS_VALUE_Y);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
}

// ---- Tocco sulla scala GS della schermata ALTM ----
// Tocco sul valore "112 km/h": km/h <-> nodi (anche la scala). Swipe sull'arco destro: sposta la
// soglia verde/gialla o gialla/rossa piu' vicina al punto in cui si appoggia il dito (passi di 5).
static unsigned long altGsTouchId = 0;
static bool altGsDragMoved = false;
static unsigned long altGsLastDrawMs = 0;

static bool altGsArcValueAt(int x, int y, float &unitVal) {
  // il riquadro SET QNH (in basso) non conta come arco: il suo tocco deve aprire SET QNH
  if (y >= SETQNH_BOX_Y - 12 && x <= SETQNH_BOX_X + SETQNH_BOX_W + 25) return false;
  const float dx = (float)x - 180.0f, dy = 180.0f - (float)y;
  if (dx < 0.0f) return false;
  const float r = sqrtf(dx * dx + dy * dy);
  if (r < 120.0f) return false;
  const float a = atan2f(dy, dx) * RAD_TO_DEG;
  if (a < -70.0f || a > 70.0f) return false;
  unitVal = (constrain(a, -60.0f, 60.0f) + 60.0f) / 120.0f * gsUnitMax();
  return true;
}
static bool altGsInReadout(int x, int y) {
  return x >= GS_READOUT_X0 - 10 && x <= GS_READOUT_X1 + 10 && y >= GS_READOUT_Y0 - 6 && y <= GS_READOUT_Y1 + 2;
}
// Trascinamento delle soglie sugli archi: vale solo se il dito si muove LUNGO l'arco. Se il primo movimento e'
// verso il centro o verso il bordo (uno swipe orizzontale che parte dall'arco), il trascinamento si annulla
// e il rilascio cambia pagina come su tutto il resto del display.
static bool arcDragIsRadial(int startX, int startY, int x, int y) {
  const float rx = (float)startX - 179.5f, ry = (float)startY - 179.5f;
  const float rl = sqrtf(rx * rx + ry * ry);
  if (rl < 1.0f) return true;
  const float mx = (float)(x - startX), my = (float)(y - startY);
  const float radial = fabsf((mx * rx + my * ry) / rl), tang = fabsf((mx * ry - my * rx) / rl);
  return radial > tang;
}
void altGsTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != MAIN_ALT_SCREEN) return;
  if (touchId != altGsTouchId) {
    altGsTouchId = touchId;
    altGsDragMoved = false;
    altGsDragTarget = 0;
    float v0;
    if (altGsArcValueAt(startX, startY, v0)) {
      const float dY = fabsf(v0 - gsKmhToUnit((float)gsYellowKmh));
      const float dR = fabsf(v0 - gsKmhToUnit((float)gsRedKmh));
      altGsDragTarget = (dY <= dR) ? 1 : 2;
    }
    if (realFlightActive() || arcBlockEnabled) altGsDragTarget = 0;   // in volo reale o con ARCH BLOCK le soglie non si cambiano
    return;
  }
  if (altGsDragTarget == 0) return;
  if (!altGsDragMoved && abs(x - startX) < 8 && abs(y - startY) < 8) return;
  if (!altGsDragMoved && arcDragIsRadial(startX, startY, x, y)) { altGsDragTarget = 0; return; }   // e' uno swipe
  altGsDragMoved = true;
  float v;
  if (!altGsArcValueAt(x, y, v)) {
    // dito oltre gli estremi dell'arco: estremo piu' vicino
    if (x < 180) return;
    v = (y > 180) ? 0.0f : gsUnitMax();
  }
  v = roundf(v / 5.0f) * 5.0f;
  const int kmh = (int)lroundf(gpsSpeedShowKnots ? v * 1.852f : v);
  const int oldY = gsYellowKmh, oldR = gsRedKmh;
  if (altGsDragTarget == 1) {   // la gialla puo' spingere avanti la rossa
    gsYellowKmh = constrain(kmh, GS_YELLOW_MIN_KMH, GS_YELLOW_MAX_KMH);
    if (gsRedKmh < gsYellowKmh + 10) gsRedKmh = gsYellowKmh + 10;
  } else gsRedKmh = constrain(kmh, gsYellowKmh + 10, GS_RED_MAX_KMH);
  if (gsUpdateAutoRange(0.0f, false)) { gsScaleNeedsErase = true; lastPrintedAlt = -99999; }   // gialla a 200 km/h: scala 0-300 subito
  if ((gsYellowKmh != oldY || gsRedKmh != oldR || altGsLastDrawMs == 0) && millis() - altGsLastDrawMs >= 50) {
    altGsLastDrawMs = millis();
    updateMainGsGauge(true);
  }
}
// Al rilascio; true = gestito qui (niente cambio pagina).
bool altGsTouchRelease(bool tapValid, int startX, int startY) {
  if (currentScreen != MAIN_ALT_SCREEN) return false;
  const bool armed = altGsDragTarget != 0;
  const bool wasDrag = armed && altGsDragMoved;
  altGsDragTarget = 0;
  altGsTouchId = 0;
  altGsDragMoved = false;
  altGsLastDrawMs = 0;
  if (wasDrag) {
    preferences.begin("altimeter", false);
    preferences.putInt("gs_yel", gsYellowKmh);
    preferences.putInt("gs_red", gsRedKmh);
    preferences.end();
    bleQueueArc();   // slave: le soglie vanno al master, che le applica e le rinvia (vale anche per GPS SPEED)
    updateMainGsGauge(true);
    playTone(1200, 50);
    return true;
  }
  if (armed) { updateMainGsGauge(true); return true; }   // tocco sull'arco senza movimento: niente cambio pagina
  if (tapValid && startX >= 92 && startX <= 268 && startY >= 110 && startY < GS_READOUT_Y0 - 6) {   // riquadro quota + mt/ft
    // Tocco sulla quota (numero grande o mt/ft): metri <-> piedi
    altitudeUnitsIsMeters = !altitudeUnitsIsMeters;
    preferences.begin("altimeter", false);
    preferences.putBool("is_meters", altitudeUnitsIsMeters);
    preferences.end();
    bleQueuePrefs();   // slave: unita' anche sul master (e da li' agli altri slave)
    playTone(1200, 50);
    redrawCurrentScreen();
    return true;
  }
  if (tapValid && altGsInReadout(startX, startY)) {
    gpsSpeedShowKnots = !gpsSpeedShowKnots;
    preferences.begin("altimeter", false);
    preferences.putBool("gs_kt", gpsSpeedShowKnots);
    preferences.end();
    bleQueuePrefs();   // slave: km/h <-> nodi anche sul master (e da li' agli altri slave)
    gsScaleNeedsErase = true;
    gsPointerDrawn = false;          // la cancellazione del settore toglie anche la lancetta
    updateMainGsGauge(true);
    lastPrintedAlt = -99999;         // il settore cancellato puo' toccare le ultime cifre della quota: ridisegno
    playTone(1200, 50);
    return true;
  }
  return false;
}

// Quota grande disegnata in uno sprite e spinta sul display in un colpo solo: niente sfarfallio e
// nessuna cancellazione di scala GS, barre vario o lancetta intorno al numero.
const int MAIN_ALT_SPR_X = 92, MAIN_ALT_SPR_Y = 113, MAIN_ALT_SPR_W = 176, MAIN_ALT_SPR_H = 89;   // stretto: il riquadro non tocca i numeri dell'arco   // fino a y=202, sopra mt/ft
static LGFX_Sprite mainAltSprite(&lcd);
static bool mainAltSpriteTried = false, mainAltSpriteOk = false;
static bool createSmallSprite(LGFX_Sprite &sp, int w, int h) {
  for (int attempt = 0; attempt < 2; ++attempt) {
    const bool usePsram = (attempt == 0);
    if (usePsram && !psramFound()) continue;
    sp.setPsram(usePsram);
    sp.setColorDepth(16);
    if (sp.createSprite(w, h) != nullptr) return true;
    sp.deleteSprite();
  }
  return false;
}
static bool drawMainAltitudeSprite(const char *txt) {
  if (!mainAltSpriteTried) {
    mainAltSpriteTried = true;
    mainAltSpriteOk = createSmallSprite(mainAltSprite, MAIN_ALT_SPR_W, MAIN_ALT_SPR_H);
    if (!mainAltSpriteOk) AVEA_LOGLN("[ALT] sprite quota non allocato: disegno diretto");
  }
  if (!mainAltSpriteOk) return false;
  const uint16_t bg = themeBg();
  const uint16_t fg = displayTheme == THEME_COLOR ? TFT_WHITE : TFT_BLACK;
  mainAltSprite.fillSprite(bg);
  const bool loaded = vlwSet(mainAltSprite, VarioNumbers80);
  float scale = 1.14f;
  const float base = loaded ? 1.0f : 0.8f;
  if (!loaded) mainAltSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
  mainAltSprite.setTextSize(scale * base, scale * base);
  const int w = mainAltSprite.textWidth(txt);
  if (w > MAIN_ALT_SPR_W - 4 && w > 0) {
    // cifre troppo larghe: carattere ridotto, centro invariato
    scale *= (float)(MAIN_ALT_SPR_W - 4) / (float)w;
    mainAltSprite.setTextSize(scale * base, scale * base);
  }
  mainAltSprite.setTextDatum(MC_DATUM);
  mainAltSprite.setTextColor(fg, bg);
  mainAltSprite.drawString(txt, MAIN_ALT_SPR_W / 2, 165 - MAIN_ALT_SPR_Y);
  if (loaded) vlwRelease(mainAltSprite);
  mainAltSprite.pushSprite(MAIN_ALT_SPR_X, MAIN_ALT_SPR_Y); drawPageLockOverlay();
  return true;
}

// Riquadro QNH giallo sotto "ALTM" (stile CHANGE LOGIN: bordo doppio, angoli raggio 8, alto 34).
// Largo 100: piu' largo toccherebbe i numeri alti dell'arco GS. "1013" grande + "hPa" piccolo.
const int ALTM_QNH_BOX_X = 130, ALTM_QNH_BOX_Y = 60, ALTM_QNH_BOX_W = 100, ALTM_QNH_BOX_H = 34;
static int altmQnhLastShown = -1;   // -1 = da ridisegnare
static void updateAltmQnhBox(bool force) {
  const int q = (int)lroundf((float)seaLevelPressure);
  if (!force && q == altmQnhLastShown) return;
  altmQnhLastShown = q;
  const int x = ALTM_QNH_BOX_X, y = ALTM_QNH_BOX_Y, w = ALTM_QNH_BOX_W, h = ALTM_QNH_BOX_H;
  lcd.fillRoundRect(x + 2, y + 2, w - 4, h - 4, 6, themeBg());   // cancella il valore vecchio
  lcd.drawRoundRect(x, y, w, h, 8, themeColor(TFT_YELLOW));
  lcd.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 7, themeColor(TFT_YELLOW));
  char num[8];
  snprintf(num, sizeof(num), "%d", q);
  setVividTextColor(TFT_YELLOW);
  lcd.setTextSize(1);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  const int wNum = lcd.textWidth(num);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSans9pt7b);
  const int wUnit = lcd.textWidth("hPa");
  const int gap = 4;
  const int x0 = x + (w - (wNum + gap + wUnit)) / 2;
  const int baseY = y + h / 2 + 8;   // linea di base comune: numero e "hPa" allineati in basso
  lcd.setTextDatum(L_BASELINE);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.drawString(num, x0, baseY);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSans9pt7b);
  lcd.drawString("hPa", x0 + wNum + gap, baseY);
  lcd.setTextDatum(MC_DATUM);
}

void drawMainScreen() {
  lcd.fillScreen(themeBg());
  lastPrintedAlt = -99999;
  gsPointerDrawn = false;
  gsLastShownKmh = -2;
  lastPrintedUnit = "";
  mainAltitudeErrorDisplayed = false;
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.5f) * THEME_TEXT_SCALE, (1.5f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("ALTM", 180, 40, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);   // in cima al display, dentro il bordo
  updateAltmQnhBox(true);   // QNH arancione sotto ALTM
  // Barre variometro spente + linea bianca dello zero; il resto lo aggiorna updateMainData().
  lastMainAltCenterX = 180;
  lastMainAltScale = 1.14f;
  lastMainVarioTenth = 99999;
  lastMainVarioTextW = 0;
  lastMainVarioSign = 0;
  lastMainVarioLit = 0;
  drawAltVarioBars(0, 0);
  drawAltVarioZeroLine();
  updateMainGsGauge(true);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.drawRect(SETQNH_BOX_X, SETQNH_BOX_Y, SETQNH_BOX_W, SETQNH_BOX_H, themeColor(TFT_YELLOW));
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  drawUiVlw("SET QNH", 180, SETQNH_BOX_Y + SETQNH_BOX_H / 2, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.62f);
}

bool resetQfeReference() {
  float currentAltitude = 0.0f;
  bool valid = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)20) == pdTRUE) {
    valid = sharedAltitudeValid && isfinite(sharedFilteredAlt);
    if (valid) currentAltitude = sharedFilteredAlt;
    xSemaphoreGive(sensorDataMutex);
  }
  if (!valid) {
    qfeReferenceValid = false;
    lastPrintedQfeMeters = -99999;
    lastPrintedQfeFeet = -99999;
    return false;
  }
  qfeReferenceAltitudeMeters = currentAltitude;
  qfeReferenceValid = true;
  lastPrintedQfeMeters = -99999;
  lastPrintedQfeFeet = -99999;
  return true;
}

// ========== PAGINA QFE: un solo stile (gli stili a nastro e analogico sono stati eliminati) ==========
// Come ALTIMETER ma specchiato: barre vario a destra, arco sinistro 0-500 m con fascia del circuito, lancetta.
// Tocco sul valore = metri <-> piedi, RESET QFE = azzera.
// Disegno completo nello sprite 360x360 della bussola e spinto in un colpo solo.
const int qfeStyle = 0;            // unico stile rimasto (A)
bool qfeShowFeet = false;          // valore grande in piedi (NVS "qfe_ft")
static uint32_t qfeLastKey = 0xFFFFFFFFu;
// Fascia "quota di circuito" dello stile A (metri sopra il campo). Si regola con uno swipe sull'arco
// destro: si sposta il bordo (basso o alto) piu' vicino al dito. Salvata in NVS ("qfe_cl", "qfe_ch").
int qfeCircuitLowM = 250;
int qfeCircuitHighM = 350;
static int qfeArcRange = 1;          // fondo scala dell'arco A = qfeArcRange * gradino (500 m / 1500 ft)
static bool qfeArcFeet = false;
static int qfeDragEdge = 0;          // 0 nessuno, 1 bordo basso, 2 bordo alto (swipe in corso)
static float qfeArcMaxU() { return qfeArcRange * (qfeShowFeet ? 1500.0f : 500.0f); }

static void qfeUiText(lgfx::LGFXBase &g, const char *txt, int x, int y, float scale, uint16_t fg, uint16_t bg,
                      decltype(MC_DATUM) datum = MC_DATUM) {
  const bool loaded = vlwSet(g, UiFont32);
  if (loaded) g.setTextSize(scale, scale);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(scale * 1.4f, scale * 1.4f); }
  g.setTextDatum(datum);
  g.setTextColor(fg, bg);
  g.drawString(txt, x, y);
  if (loaded) vlwRelease(g);
}
static int qfeUiWidth(lgfx::LGFXBase &g, const char *txt, float scale) {
  const bool loaded = vlwSet(g, UiFont32);
  if (loaded) g.setTextSize(scale, scale);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(scale * 1.4f, scale * 1.4f); }
  const int w = g.textWidth(txt);
  if (loaded) vlwRelease(g);
  return w;
}
static void qfeBigNumber(lgfx::LGFXBase &g, const char *txt, int cx, int cy, float scale, int maxW, uint16_t fg, uint16_t bg) {
  const bool loaded = vlwSet(g, VarioNumbers80);
  const float base = loaded ? 1.0f : 0.8f;
  if (!loaded) g.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
  g.setTextSize(scale * base, scale * base);
  const int w = g.textWidth(txt);
  if (w > maxW && w > 0) { scale *= (float)maxW / (float)w; g.setTextSize(scale * base, scale * base); }
  g.setTextDatum(MC_DATUM);
  (void)bg;
  g.setTextColor(fg);   // sfondo trasparente: nessun riquadro nero che tocca i numeri dell'arco
  g.drawString(txt, cx, cy);
  if (loaded) vlwRelease(g);
}
// "1024 ft": numero nel colore fg, unita' in ciano, centrato su cx
static void qfeValueUnit(lgfx::LGFXBase &g, const char *num, const char *unit, int cx, int y, float sNum, float sUnit,
                         uint16_t fg, uint16_t bg) {
  const int wN = qfeUiWidth(g, num, sNum), wU = qfeUiWidth(g, unit, sUnit), gap = 6;
  const int x = cx - (wN + gap + wU) / 2;
  qfeUiText(g, num, x, y, sNum, fg, bg, ML_DATUM);
  qfeUiText(g, unit, x + wN + gap, y + 1, sUnit, themeColor(TFT_CYAN), bg, ML_DATUM);
}
static void qfeResetButton(lgfx::LGFXBase &g, int x, int y, int w, int h, bool rounded, float s, uint16_t bg) {
  const uint16_t c = themeColor(TFT_YELLOW);
  if (rounded) { g.drawRoundRect(x, y, w, h, h / 2, c); g.drawRoundRect(x + 1, y + 1, w - 2, h - 2, h / 2 - 1, c); }
  else { g.drawRect(x, y, w, h, c); g.drawRect(x + 1, y + 1, w - 2, h - 2, c); }
  qfeUiText(g, "RESET QFE", x + w / 2, y + h / 2, s, c, bg);
}
// Lancetta triangolare degli archi: arancione con contorno giallo (ALTIMETER, QFE, GPS SPEED).
void drawArcPointerTri(lgfx::LGFXBase &g, int x0, int y0, int x1, int y1, int x2, int y2) {
  g.fillTriangle(x0, y0, x1, y1, x2, y2, themeVivid(TFT_ORANGE));
  g.drawTriangle(x0, y0, x1, y1, x2, y2, themeVivid(TFT_YELLOW));
}
// QFE: arco sul lato sinistro = arco dell'ALTIMETER specchiato (x -> 360 - x)
static void qfePolar(float r, float deg, float off, int &x, int &y) { gsPolar(r, deg, off, x, y); x = 360 - x; }
__attribute__((unused)) static void qfeTri(lgfx::LGFXBase &g, float deg, float rTip, float rBase, float half, uint16_t color) {
  int x0, y0, x1, y1, x2, y2;
  qfePolar(rTip, deg, 0, x0, y0);
  qfePolar(rBase, deg, half, x1, y1);
  qfePolar(rBase, deg, -half, x2, y2);
  (void)color;
  drawArcPointerTri(g, x0, y0, x1, y1, x2, y2);
}

static void renderQfe(bool valid, const char *msg, float qfeM, bool varioValid, float vario) {
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  const uint16_t bg = themeBg();
  const uint16_t white = themeColor(TFT_WHITE), cyan = themeColor(TFT_CYAN), grey = themeColor(TFT_LIGHTGREY);
  g.fillScreen(bg);
  const float k = qfeShowFeet ? 3.28084f : 1.0f;
  const float hU = qfeM * k;                                  // altezza nell'unita' mostrata
  const int mainVal = (int)lroundf(hU);
  const int otherVal = (int)lroundf(qfeShowFeet ? qfeM : qfeM * 3.28084f);
  const char *mainUnit = qfeShowFeet ? "ft" : "mt";
  const char *otherUnit = qfeShowFeet ? "mt" : "ft";
  char bufMain[16], bufOther[16];
  snprintf(bufMain, sizeof(bufMain), "%d", mainVal);
  snprintf(bufOther, sizeof(bufOther), "%d", otherVal);
  const uint16_t msgColor = themeVivid(TFT_RED);

  if (qfeStyle == 0) {
    // ---------- A: stile ALTM ----------
    int sign = 0, lit = 0, whole = 0;
    if (varioValid) {
      whole = (int)lroundf(constrain(vario, -99.0f, 99.0f));
      if (whole != 0) { sign = whole > 0 ? 1 : -1; lit = min(abs(whole), ALT_VARIO_BARS * 2); }
    }
    arcTarget = &g;
    altVarioBarsRight = true;    // QFE: barre del vario a destra
    drawAltVarioBars(sign, lit);
    altVarioBarsRight = false;
    arcTarget = &lcd;
    if (varioValid) {
      char vb[8];
      if (whole == 0) snprintf(vb, sizeof(vb), "0"); else snprintf(vb, sizeof(vb), "%+d", whole);
      const uint16_t green = altVarioUpColor();
      g.setFreeFont(&lgfx::v1::fonts::FreeSans18pt7b);
      g.setTextSize(1.0f, 1.0f);
      g.setTextDatum(MR_DATUM);   // valore accanto alle barre, sul lato destro
      g.setTextColor(green, bg);
      g.drawString(vb, 360 - ALT_VARIO_TEXT_X, 178);
      g.setFreeFont(&lgfx::v1::fonts::FreeSans12pt7b);
      g.setTextSize(0.65f * THEME_TEXT_SCALE, 0.65f * THEME_TEXT_SCALE);
      g.drawString("m/s", 360 - ALT_VARIO_TEXT_X - 5, 205);
    }
    // arco sinistro: fondo scala automatico a gradini di 500 m (1500 ft). Si allarga appena l'altezza
    // supera il fondo scala e torna indietro quando rientra con margine (isteresi 20% del gradino).
    const float stepU = qfeShowFeet ? 1500.0f : 500.0f;
    if (qfeArcFeet != qfeShowFeet) { qfeArcFeet = qfeShowFeet; qfeArcRange = 1; }
    if (valid && qfeDragEdge == 0) {   // durante lo swipe la scala resta ferma
      while (hU > qfeArcRange * stepU && qfeArcRange < 20) qfeArcRange++;
      while (qfeArcRange > 1 && hU < (qfeArcRange - 1) * stepU - 0.2f * stepU) qfeArcRange--;
    }
    const float maxU = qfeArcRange * stepU;
    const int major = (int)(maxU / 5.0f), tick = major / 4;
    auto ang = [&](float u) { return -60.0f + 120.0f * constrain(u, 0.0f, maxU) / maxU; };
    arcTarget = &g;
    // Come l'arco GS dell'ALTIMETER: fascia 162-172 verde, quota di circuito in giallo
    fillArcAA(g, 179.5f, 179.5f, 162.0f, ARC_R_OUT, -60.0f, 60.0f, true, arcSafeColor(), themeBg());   // lato sinistro, bordi sfumati
    fillArcAA(g, 179.5f, 179.5f, 162.0f, ARC_R_OUT, ang(qfeCircuitLowM * k), ang(qfeCircuitHighM * k), true, themeVivid(TFT_YELLOW), themeBg());
    arcTarget = &lcd;
    // Trattini neri (4 px) alle estremita' della fascia gialla
    for (int e = 0; e < 2; ++e) {
      const float ev = (e == 0 ? qfeCircuitLowM : qfeCircuitHighM) * k;
      if (ev <= 0.0f || ev >= maxU) continue;
      int x0, y0, x1, y1;
      qfePolar(161.0f, ang(ev), 0, x0, y0);
      qfePolar(180.0f, ang(ev), 0, x1, y1);
      drawThickLineOn(g, x0, y0, x1, y1, TFT_BLACK, 4);   // trattino nero 4 px
    }
    if (qfeDragEdge != 0) {   // bordo in regolazione evidenziato
      int x0, y0, x1, y1;
      const float ea = ang((qfeDragEdge == 1 ? qfeCircuitLowM : qfeCircuitHighM) * k);
      qfePolar(156.0f, ea, 0, x0, y0);
      qfePolar(178.0f, ea, 0, x1, y1);
      g.drawWideLine(x0, y0, x1, y1, 2.5f, white);
    }
    if (valid) arcSweepC(g, 2, 0, 0, ang(0.0f), ang(hU), 131.0f, 161.0f, bg);   // lancetta proposta C (sotto tacche e numeri)
    for (int v = 0; v <= (int)maxU; v += tick) {
      int x0, y0, x1, y1;
      qfePolar(v % major == 0 ? 142.0f : 151.0f, ang((float)v), 0, x0, y0);   // 20 tacche, 5 lunghe (come ALTIMETER)
      qfePolar(161.0f, ang((float)v), 0, x1, y1);
      drawThickLineOn(g, x0, y0, x1, y1, white, 2);   // stesso spessore delle tacche ALTIMETER
    }
    g.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    g.setTextSize(1.1f * THEME_TEXT_SCALE, 1.1f * THEME_TEXT_SCALE);   // numeri dell'arco +30%
    g.setTextDatum(MC_DATUM);
    g.setTextColor(grey, bg);
    for (int v = major; v <= (int)maxU; v += major) {
      int lx, ly;
      const float la = ang((float)v);
      qfePolar(fabsf(la) < 15.0f ? 120.0f : 126.0f, la, 0, lx, ly);
      char b[8];
      if (v >= 10000) snprintf(b, sizeof(b), "%dk", v / 1000); else snprintf(b, sizeof(b), "%d", v);
      g.drawString(b, lx, ly);
    }
    // (il triangolo della lancetta e' sostituito dalla scia disegnata sotto tacche e numeri)
    qfeUiText(g, "QFE", 180, 40, 0.95f, cyan, bg);
    if (valid) qfeBigNumber(g, bufMain, 180, 150, 1.14f, 200, white, bg);
    else qfeUiText(g, msg, 180, 150, 0.8f, msgColor, bg);
    qfeUiText(g, mainUnit, 180, 205, 0.8f, cyan, bg);
    if (valid) qfeValueUnit(g, bufOther, otherUnit, 180, 246, 0.85f, 0.62f, white, bg);
    g.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    g.setTextSize(0.75f * THEME_TEXT_SCALE, 0.75f * THEME_TEXT_SCALE);
    g.setTextDatum(MC_DATUM);
    if (qfeDragEdge != 0) {
      char cb[32];
      snprintf(cb, sizeof(cb), "CIRCUIT %d-%d %s", (int)lroundf(qfeCircuitLowM * k), (int)lroundf(qfeCircuitHighM * k), mainUnit);
      g.setTextColor(themeVivid(TFT_YELLOW), bg);
      g.drawString(cb, 180, 276);
    } else {
      g.setTextColor(themeColor(TFT_DARKGREY), bg);
      g.drawString("ABOVE FIELD", 180, 276);
    }
    qfeResetButton(g, 115, 296, 130, 34, false, 0.55f, bg);
    if (valid) arcNeedleC(g, 2, 0, 0, ang(hU), 131.0f, 161.0f, bg);   // lancetta in primo piano, sopra tacche e scritte
  }
  g.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

void drawQfeScreen() {
  lastPrintedQfeMeters = -99999;
  lastPrintedQfeFeet = -99999;
  lastQfeInvalidMessage[0] = '\0';
  qfeLastKey = 0xFFFFFFFFu;
  if (!ensureNavigationSprite()) lcd.fillScreen(themeBg());
  updateQfeData();
}

// Ridisegna solo quando cambia qualcosa di visibile (valore intero, vario, stato, stile, unita').
void updateQfeData() {
  if (currentScreen != QFE_SCREEN) return;
  float localAlt = 0.0f, vario = 0.0f;
  bool valid = false, varioValid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    localAlt = sharedFilteredAlt;
    valid = sharedAltitudeValid && isfinite(localAlt);
    vario = sharedVarioKalman;
    varioValid = sharedVarioValid && isfinite(vario);
    xSemaphoreGive(sensorDataMutex);
  }
  const char *msg = !qfeReferenceValid ? "QFE NOT SET" : (!valid ? "ALT INVALID" : "");
  const bool ok = valid && qfeReferenceValid;
  const float qfeM = ok ? localAlt - qfeReferenceAltitudeMeters : 0.0f;
  const int shown = ok ? (int)lroundf(qfeM * (qfeShowFeet ? 3.28084f : 1.0f)) : 0;
  const int vz = varioValid ? (int)lroundf(constrain(vario, -15.0f, 15.0f)) : 99;
  const uint32_t key = ((uint32_t)(shown & 0xFFFF)) | ((uint32_t)((vz + 32) & 0x7F) << 16) |
                       ((uint32_t)(qfeStyle & 3) << 23) | ((uint32_t)qfeShowFeet << 25) |
                       ((uint32_t)ok << 26) | ((uint32_t)qfeReferenceValid << 27) | ((uint32_t)displayTheme << 28);
  if (key == qfeLastKey) return;
  qfeLastKey = key;
  renderQfe(ok, msg, qfeM, varioValid, vario);
}

// ---- Swipe sull'arco destro (stile A): regola la fascia del circuito ----
static unsigned long qfeDragTouchId = 0;
static bool qfeDragMoved = false;
static unsigned long qfeDragDrawMs = 0;
static bool qfeArcValueAt(int x, int y, float &u) {
  // il riquadro RESET QFE (in basso) non conta come arco; l'arco e' sul lato sinistro
  if (y >= 286 && x >= 98) return false;
  const float dx = 180.0f - (float)x, dy = 180.0f - (float)y;
  if (dx < 0.0f || sqrtf(dx * dx + dy * dy) < 120.0f) return false;
  const float a = atan2f(dy, dx) * RAD_TO_DEG;
  if (a < -70.0f || a > 70.0f) return false;
  u = (constrain(a, -60.0f, 60.0f) + 60.0f) / 120.0f * qfeArcMaxU();
  return true;
}
void qfeTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != QFE_SCREEN || qfeStyle != 0) return;
  const float k = qfeShowFeet ? 3.28084f : 1.0f;
  if (touchId != qfeDragTouchId) {
    qfeDragTouchId = touchId;
    qfeDragMoved = false;
    qfeDragEdge = 0;
    float u0;
    if (qfeArcValueAt(startX, startY, u0)) {
      qfeDragEdge = (fabsf(u0 - qfeCircuitLowM * k) <= fabsf(u0 - qfeCircuitHighM * k)) ? 1 : 2;
    }
    if (realFlightActive() || arcBlockEnabled) qfeDragEdge = 0;   // in volo reale o con ARCH BLOCK la fascia non si cambia
    return;
  }
  if (qfeDragEdge == 0) return;
  if (!qfeDragMoved && abs(x - startX) < 8 && abs(y - startY) < 8) return;
  if (!qfeDragMoved && arcDragIsRadial(startX, startY, x, y)) { qfeDragEdge = 0; qfeLastKey = 0xFFFFFFFFu; return; }   // e' uno swipe
  qfeDragMoved = true;
  float u;
  if (!qfeArcValueAt(x, y, u)) {
    if (x > 180) return;   // arco a sinistra
    u = (y > 180) ? 0.0f : qfeArcMaxU();
  }
  const float stepU = qfeShowFeet ? 50.0f : 10.0f;   // passi di 10 m oppure 50 ft
  u = roundf(u / stepU) * stepU;
  const int m = (int)lroundf(u / k);
  if (qfeDragEdge == 1) qfeCircuitLowM = constrain(m, 0, qfeCircuitHighM - 20);
  else qfeCircuitHighM = constrain(m, qfeCircuitLowM + 20, 9000);
  if (millis() - qfeDragDrawMs >= 50) {
    qfeDragDrawMs = millis();
    qfeLastKey = 0xFFFFFFFFu;
    updateQfeData();
  }
}
// Al rilascio; true = gestito (niente cambio pagina, niente tap).
bool qfeTouchRelease() {
  if (currentScreen != QFE_SCREEN) return false;
  const bool armed = qfeDragEdge != 0;
  const bool moved = armed && qfeDragMoved;
  qfeDragEdge = 0;
  qfeDragTouchId = 0;
  qfeDragMoved = false;
  if (!armed) return false;
  if (moved) {
    preferences.begin("altimeter", false);
    preferences.putInt("qfe_cl", qfeCircuitLowM);
    preferences.putInt("qfe_ch", qfeCircuitHighM);
    preferences.end();
    bleQueuePrefs();   // slave: fascia di circuito anche sul master
    playTone(1200, 50);
  }
  qfeLastKey = 0xFFFFFFFFu;
  updateQfeData();
  return true;
}

// Tocco (tap) sulla pagina QFE. true = gestito.
bool qfeHandleTap(int x, int y) {
  // zona RESET QFE (dipende dallo stile)
  const bool resetZone = (x >= 100 && x <= 260 && y >= 286 && y <= 345);
  const bool valueZone = (x >= 70 && x <= 290 && y >= 95 && y <= 270);
  if (resetZone) {
    resetQfeReference();
    bleQueuePrefs();   // slave: RESET QFE anche sul master (stessa quota di riferimento)
    drawQfeScreen();
    return true;
  }
  if (valueZone) {   // il tocco sul titolo non cambia piu' stile
    qfeShowFeet = !qfeShowFeet;
    preferences.begin("altimeter", false);
    preferences.putBool("qfe_ft", qfeShowFeet);
    preferences.end();
    bleQueuePrefs();   // slave: metri <-> piedi del QFE anche sul master
    drawQfeScreen();
    return true;
  }
  return false;
}

void drawQnhSetupScreen() {
  lcd.fillScreen(themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("SET QNH", 180, 45, MC_DATUM, themeColor(TFT_WHITE), themeBg(), THEME_TEXT_SCALE);
  lcd.drawRect(40, 80, 130, 60, themeColor(TFT_GREEN));
  drawUiVlw("DOWN -", 105, 110, MC_DATUM, themeColor(TFT_GREEN), themeBg(), 0.82f);
  lcd.drawRect(190, 80, 130, 60, themeColor(TFT_GREEN));
  drawUiVlw("UP +", 255, 110, MC_DATUM, themeColor(TFT_GREEN), themeBg(), 0.82f);
  char qnhBuf[32];
  snprintf(qnhBuf, sizeof(qnhBuf), "%d hPa", (int)round(seaLevelPressure));
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
  lcd.setTextSize(1, 1);
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString(qnhBuf, 180, 170);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  int mColor = qnhPendingMeters ? TFT_GREEN : TFT_YELLOW;    // non selezionato: giallo (prima grigio)
  int fColor = !qnhPendingMeters ? TFT_GREEN : TFT_YELLOW;
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  if (displayTheme != THEME_COLOR && qnhPendingMeters) {
    // Temi bianco/nero e giallo/nero: riquadro selezionato invertito (fondo nero, scritta color sfondo).
    lcd.fillRect(40, 215, 130, 50, TFT_BLACK);
    lcd.setTextColor(themeBg(), TFT_BLACK);
  } else {
    lcd.drawRect(40, 215, 130, 50, themeColor(mColor));
    setThemeTextColor(mColor, TFT_BLACK);
  }
  lcd.drawString("METERS", 105, 240);
  if (displayTheme != THEME_COLOR && !qnhPendingMeters) {
    lcd.fillRect(190, 215, 130, 50, TFT_BLACK);
    lcd.setTextColor(themeBg(), TFT_BLACK);
  } else {
    lcd.drawRect(190, 215, 130, 50, themeColor(fColor));
    setThemeTextColor(fColor, TFT_BLACK);
  }
  lcd.drawString("FEET", 255, 240);
  lcd.drawRect(80, 275, 200, 40, themeColor(TFT_YELLOW));
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString(qnhBootMode ? "NEXT  >" : "BACK / SAVE", 180, 295);
  // QNH arrivato dal telefono (app AVEA Link): e' solo una proposta. L'altimetro non si muove
  // da solo: il valore sale sopra solo se lo tocchi. Dopo 10 minuti diventa grigio, dopo mezz'ora
  // sparisce. Se il telefono non c'e', qui non compare niente e la pagina resta come sempre.
  const bool metShow = metQnhValid && metFresh()
                       && fabsf((float)metQnh - (float)seaLevelPressure) >= 0.5f;
  if (metShow) {
    char mb[40];
    snprintf(mb, sizeof(mb), "PHONE %d hPa", (int)lroundf((float)metQnh));
    const int age = metAgeMin();
    const uint16_t c = (age >= 0 && age <= 10) ? themeColor(TFT_GREEN) : themeColor(TFT_DARKGREY);
    drawUiVlw(mb, 180, 336, MC_DATUM, c, themeBg(), 0.50f);
  }
  if (qnhBootMode && bootLitersWanted()) {   // accensione: passo 1 di 2 (poi SET LITERS)
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSans9pt7b);
    lcd.setTextSize(1);
    setThemeTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.drawString("1 / 2", 180, metShow ? 353 : 334);
  }
}

// Linea sottile grigia tra "m/s" e la quota (solo estetica). Va ridisegnata dopo ogni pulizia della zona del numero.
static void drawVarioSeparatorLine() {
  lcd.fillRect(80, 263, 200, 2, themeColor(TFT_DARKGREY));
}

void drawVariometerScreen() {
  lcd.fillScreen(themeBg());
  lastPrintedVario = -99999;
  lastVarioDirection = 99;
  lastPrintedVarioAlt = VARIO_ALT_FORCE_REDRAW;
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.5f) * THEME_TEXT_SCALE, (1.5f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("VARIO", 180, 65, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);   // 10 px piu' in alto
  drawVarioSeparatorLine();
}

int daysInMonth(int year, int month) {
  if (month == 2) { bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)); return leap ? 29 : 28; }
  if (month == 4 || month == 6 || month == 9 || month == 11) return 30;
  return 31;
}

// Aggiorna la posizione usata per il fuso automatico (dal loop, ogni 5 s). Lo slave usa il GPS del master.
void updateAutoTimezonePosition() {
  static unsigned long lastMs = 0;
  const unsigned long now = millis();
  if (lastMs != 0 && now - lastMs < 5000UL) return;
  lastMs = now;
  double lat = NAN, lon = NAN;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
    if (gpsDataValid) { lat = gps_latitude; lon = gps_longitude; }
    xSemaphoreGive(sensorDataMutex);
  }
  if (!isfinite(lat) && homePositionValid && !demoHomeAutoSet) { lat = homeLatitude; lon = homeLongitude; }
  if (!isfinite(lat) || !isfinite(lon) || (lat == 0.0 && lon == 0.0)) return;   // nessuna posizione: resta l'ultima
  tzAutoLat = lat;
  tzAutoLon = lon;
  const int rule = tzaRuleAt(lat, lon);
  if (rule != tzAutoRule) {   // cambio di zona (o primo fix): salva la posizione per i prossimi avvii senza GPS
    tzAutoRule = rule;
    preferences.begin("altimeter", false);
    preferences.putDouble("tz_lat", lat);
    preferences.putDouble("tz_lon", lon);
    preferences.end();
    lastPrintedClockKey = 0xFFFFFFFFUL;   // ridisegna l'orologio
    AVEA_LOGF("[TZ] zona %d (%.3f, %.3f): %s\n", rule, lat, lon, rule > 0 ? TZA_RULES[rule] : "nautica");
  }
}

void syncSoftwareClockFromGps(int hour, int minute, int second, int day, int month, int year) {
  if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59 ||
      year < 2000 || month < 1 || month > 12 || day < 1 || day > daysInMonth(year, month)) return;
  softwareClockHour = hour;
  softwareClockMinute = minute;
  softwareClockSecond = second;
  softwareClockDay = day;
  softwareClockMonth = month;
  softwareClockYear = year;
  softwareClockTickMillis = millis();
  softwareClockLastGpsSyncMillis = softwareClockTickMillis;
  softwareClockInitialized = true;
}

void advanceSoftwareClock() {
  if (!softwareClockInitialized) return;
  unsigned long now = millis();
  unsigned long elapsedSeconds = (now - softwareClockTickMillis) / 1000UL;
  if (elapsedSeconds == 0) return;
  softwareClockTickMillis += elapsedSeconds * 1000UL;

  unsigned long secondsOfDay = (unsigned long)softwareClockHour * 3600UL +
                               (unsigned long)softwareClockMinute * 60UL +
                               (unsigned long)softwareClockSecond + elapsedSeconds;
  unsigned long elapsedDays = secondsOfDay / 86400UL;
  secondsOfDay %= 86400UL;
  softwareClockHour = (int)(secondsOfDay / 3600UL);
  softwareClockMinute = (int)((secondsOfDay % 3600UL) / 60UL);
  softwareClockSecond = (int)(secondsOfDay % 60UL);
  while (elapsedDays > 0) {
    softwareClockDay++;
    if (softwareClockDay > daysInMonth(softwareClockYear, softwareClockMonth)) {
      softwareClockDay = 1;
      softwareClockMonth++;
      if (softwareClockMonth > 12) {
        softwareClockMonth = 1;
        softwareClockYear++;
      }
    }
    elapsedDays--;
  }
}

bool getLocalClock(int &hour, int &minute, int &second, int &day, int &month, int &year) {
  advanceSoftwareClock();
  if (!softwareClockInitialized) return false;
  // Ora locale automatica: fuso e ora legale dalla posizione (TimeZoneAuto.h), calcolati sull'istante UTC.
  const int64_t utc = tzaUtcEpoch(softwareClockYear, softwareClockMonth, softwareClockDay,
                                  softwareClockHour, softwareClockMinute, softwareClockSecond);
  bool dst = false;
  char abbr[12];
  const int32_t off = tzaOffsetSeconds(tzAutoLat, tzAutoLon, utc, dst, nullptr, abbr, sizeof(abbr));
  tzAutoOffsetSec = off;
  tzAutoIsDst = dst;
  memcpy(tzAutoAbbr, abbr, sizeof(tzAutoAbbr));
  const int64_t loc = utc + off;
  const int32_t locDays = (int32_t)(loc >= 0 ? loc / 86400LL : (loc - 86399LL) / 86400LL);
  const int32_t secOfDay = (int32_t)(loc - (int64_t)locDays * 86400LL);
  int32_t ly; int lm, ld;
  tzaCivilFromDays(locDays, ly, lm, ld);
  year = (int)ly; month = lm; day = ld;
  hour = secOfDay / 3600;
  minute = (secOfDay % 3600) / 60;
  second = secOfDay % 60;

  // In demo only, advance simulated time at 60x: one simulated minute per real second.
  // The base is set to sunset - 31 min; leaving demo restores the real software clock.
  if (demoMode && demoSunsetClockInitialized) {
    const unsigned long elapsedMs = millis() - demoSunsetClockStartMillis;
    long totalMinutes = (long)demoSunsetBaseMinute + (long)(elapsedMs / 1000UL);
    while (totalMinutes >= 1440L) {
      totalMinutes -= 1440L;
      day++;
      if (day > daysInMonth(year, month)) {
        day = 1;
        month++;
        if (month > 12) { month = 1; year++; }
      }
    }
    while (totalMinutes < 0L) {
      totalMinutes += 1440L;
      day--;
      if (day < 1) {
        month--;
        if (month < 1) { month = 12; year--; }
        day = daysInMonth(year, month);
      }
    }
    hour = (int)(totalMinutes / 60L);
    minute = (int)(totalMinutes % 60L);
    second = (int)((elapsedMs % 1000UL) * 60UL / 1000UL);
  }
  return true;
}

void getLocalTimeText(char *out, size_t outSize) {
  int hour, minute, second, day, month, year;
  if (getLocalClock(hour, minute, second, day, month, year)) {
    snprintf(out, outSize, "%02d:%02d:%02d", hour, minute, second);
  } else {
    snprintf(out, outSize, "--:--:--");
  }
}

void drawAvSetupScreen() {
  lcd.fillScreen(themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.35f) * THEME_TEXT_SCALE, (1.35f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  lcd.drawString("SETUP", 180, 62);

  // Un solo controllo DUAL MODE sostituisce le quattro caselle precedenti.
  const int dualX = 25, dualY = 90, dualW = 300, dualH = 86;
  lcd.drawRect(dualX, dualY, dualW, dualH, themeColor(TFT_GREEN));
  setThemeTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.drawString(dualModeEnabled ? "DUAL MODE: ON" : "DUAL MODE: OFF",
                 dualX + dualW / 2, dualY + dualH / 2);

  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString("Vario activation threshold (m/s)", 180, 198);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  const int tw = 65, th = 52, spacing = 8, startX = 33, ty = 214;
  for (int i = 0; i < 4; ++i) {
    int x = startX + i * (tw + spacing);
    bool selected = (i == avThresholdIndex);
    if (selected && displayTheme != THEME_COLOR) {
      // Temi bianco/nero e giallo/nero: riquadro selezionato invertito.
      lcd.fillRect(x, ty, tw, th, TFT_BLACK);
      lcd.setTextColor(themeBg(), TFT_BLACK);
    } else {
      lcd.drawRect(x, ty, tw, th, themeColor(selected ? TFT_GREEN : TFT_DARKGREY));
      setThemeTextColor(selected ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
    }
    char thresholdBuf[8];
    dtostrf(avLiftThresholds[i], 3, 1, thresholdBuf);
    lcd.drawString(thresholdBuf, x + tw / 2, ty + th / 2);
  }

  // Riquadro EXIT: stesso carattere e grandezza di "DUAL MODE"
  const int exitX = 100, exitY = 278, exitW = 160, exitH = 40;
  lcd.drawRect(exitX, exitY, exitW, exitH, themeColor(TFT_GREEN));
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.setTextDatum(MC_DATUM);
  setThemeTextColor(TFT_GREEN, TFT_BLACK);
  lcd.drawString("EXIT", exitX + exitW / 2, exitY + exitH / 2);
}

void drawVarioFilterScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize(1.25f * THEME_TEXT_SCALE, 1.25f * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("FILTER", 180, 42);

  const char* labels[] = { "SMOOTH", "NORMAL", "REACTIVE" };
  const VarioFilterLevel levels[] = { FILTER_SMOOTH, FILTER_NORMAL, FILTER_REACTIVE };
  const int buttonX = 45, buttonW = 270, buttonH = 52, firstY = 88, stepY = 64;
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize(1.0f * THEME_TEXT_SCALE, 1.0f * THEME_TEXT_SCALE);
  for (int i = 0; i < 3; ++i) {
    const int buttonY = firstY + i * stepY;
    const bool selected = varioFilterLevel == levels[i];
    const uint16_t accent = selected ? themeColor(TFT_GREEN) : themeColor(TFT_DARKGREY);
    if (selected) lcd.fillRoundRect(buttonX, buttonY, buttonW, buttonH, 8, accent);
    else lcd.drawRoundRect(buttonX, buttonY, buttonW, buttonH, 8, accent);
    if (selected) {
      const uint16_t selectedText = displayTheme == THEME_COLOR ? TFT_BLACK : themeBg();
      lcd.setTextColor(selectedText, accent);
    } else {
      setThemeTextColor(TFT_WHITE, TFT_BLACK);
    }
    lcd.drawString(labels[i], 180, buttonY + buttonH / 2);
  }
}

const ScreenState dualPageOptions[8] = {
  MAIN_ALT_SCREEN, VARIOMETER_SCREEN, COMPASS_SCREEN, METEO_SCREEN,
  QFE_SCREEN, GPS_SPEED_SCREEN, GPS_CLOCK_SCREEN, GO_TO_HOME_SCREEN
};

const char* dualPageLabels[8] = {
  "ALTIMETER", "VARIOMETER", "COMPASS", "SENSOR",
  "QFE", "GPS-SPEED", "GPS-CLOCK", "GO TO HOME"
};

int dualOptionX(int index) { return index < 4 ? 10 : 185; }
int dualOptionY(int index) { return 82 + (index % 4) * 53; }

bool isDualSelected(ScreenState page) {
  return (dualSelectionCount > 0 && dualSelection[0] == page) ||
         (dualSelectionCount > 1 && dualSelection[1] == page);
}

void saveDualPreferences() {
  preferences.begin("altimeter", false);
  preferences.putBool("av_dual", dualModeEnabled);
  preferences.putInt("dual_count", dualSelectionCount);
  preferences.putInt("dual_page_0", (int)dualSelection[0]);
  preferences.putInt("dual_page_1", (int)dualSelection[1]);
  preferences.end();
}

void drawDualModeScreen() {
  lcd.fillScreen(themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.25f) * THEME_TEXT_SCALE, (1.25f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("DUAL MODE", 180, 42, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.95f * THEME_TEXT_SCALE);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((0.95f) * THEME_TEXT_SCALE, (0.95f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString("SELECT 2 PAGES", 180, 64);

  for (int i = 0; i < 8; ++i) {
    int x = dualOptionX(i), y = dualOptionY(i);
    bool selected = isDualSelected(dualPageOptions[i]);
    if (selected && displayTheme != THEME_COLOR) {
      // Temi bianco/nero e giallo/nero: riquadro selezionato invertito
      // (sfondo nero, scritta del colore dello sfondo della pagina).
      lcd.fillRect(x, y, 165, 40, TFT_BLACK);
      lcd.setTextColor(themeBg(), TFT_BLACK);
    } else {
      lcd.drawRect(x, y, 165, 40, themeColor(selected ? TFT_GREEN : TFT_DARKGREY));
      setThemeTextColor(selected ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
    }
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    lcd.drawString(dualPageLabels[i], x + 82, y + 20);
  }
  lcd.setTextSize((0.85f) * THEME_TEXT_SCALE, (0.85f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(dualSelectionCount == 2 ? "SWIPE TO EXIT DUAL" : "TAP TWO PAGES", 180, 310);
}

void updateDualMode() {
  if (!dualModeEnabled || dualSelectionCount != 2) return;
  // Gli avvisi a tutto schermo non vanno sostituiti dal cambio pagina automatico.
  if (currentScreen == RISK_WARNING_SCREEN || currentScreen == ICT_ALERT_SCREEN ||
      currentScreen == FLIGHT_END_SCREEN || currentScreen == SUNSET_SCREEN ||
      currentScreen == FUEL_RESERVE_SCREEN || currentScreen == LICENSE_SCREEN ||
      currentScreen == AIRSPACE_ALERT_SCREEN || currentScreen == DENSITY_ALT_SCREEN) {   // l'avviso RESERVE resta fino al tocco
    dualLastSwitchMillis = millis();
    return;
  }
  unsigned long now = millis();
  if (now - dualLastSwitchMillis >= DUAL_SWITCH_INTERVAL_MS) {
    dualLastSwitchMillis = now;
    dualActiveIndex = (dualActiveIndex + 1) % 2;
    currentScreen = dualSelection[dualActiveIndex];
    redrawCurrentScreen();
  }
}

// ========== GPS SPEED: strumento ad arco ==========
// Arco di 270 gradi: 0 in basso a sinistra (225 gradi), fondo scala in basso a destra (-45), senso orario.
// Fascia verde/gialla/rossa (soglie regolabili con swipe sull'arco), tacche, scritte, lancetta bianca,
// triangolino arancione della massima. Al centro: valore grande, unita', l'altra unita' piu' piccola.
// Tocco al centro: scambia km/h <-> nodi (anche la scala). Tutto disegnato nello sprite 360x360.
static int gpsSpeedDragTarget = 0;            // 0 = nessuno, 1 = soglia gialla, 2 = soglia rossa
static unsigned long gpsSpeedDragTouchId = 0; // touchStartAtMillis del tocco in corso
static bool gpsSpeedDragMoved = false;
static unsigned long gpsSpeedLastDragDrawMs = 0;
static uint32_t gpsSpeedLastKey = 0xFFFFFFFFu;

static float gpsScaleMax() { return gsScaleMaxFor(gpsSpeedShowKnots, gsRangeIdx); }   // fondo scala automatico, come ALTIMETER
static float gpsToUnit(float kmh) { return gpsSpeedShowKnots ? kmh / 1.852f : kmh; }
static float gpsArcAngle(float unitVal) { return 225.0f - 270.0f * constrain(unitVal, 0.0f, gpsScaleMax()) / gpsScaleMax(); }
static void gpsArcPt(float r, float deg, float off, int &x, int &y) {
  const float a = deg * DEG_TO_RAD;
  x = (int)lroundf(180.0f + r * cosf(a) - off * sinf(a));
  y = (int)lroundf(180.0f - r * sinf(a) - off * cosf(a));
}
// GPS SPEED: numeri ogni 50 km/h anche sulla scala 0-300 (ogni 100 oltre); in nodi ogni 25 fino a 110, poi ogni 50
static int gpsLabelStepFor(bool kt, float maxU) { return gsScaleIs150(kt, maxU) ? 25 : (kt ? (maxU <= 110.5f ? 25 : 50) : (maxU <= 300.5f ? 50 : 100)); }
static int gpsTickStepFor(bool kt, float maxU) { return gsScaleIs150(kt, maxU) ? 5 : (kt ? (maxU <= 160.5f ? 5 : 10) : (maxU <= 300.5f ? 10 : 20)); }
// Scia al posto della lancetta (come nei cruscotti moderni): l'arco tra rIn e rOut si riempie da 0 alla velocita'
// con un velo bianco sempre piu' luminoso verso la punta, e una linea arancione luminosa segna il valore.
// Il fondo e' a tinta unita, quindi la "trasparenza" e' un colore mescolato con il fondo, spicchio per spicchio.
// Sui temi chiari il velo e' grigio scuro invece che bianco.
static void gpsDrawSpeedSweep(lgfx::LGFXBase &g, float unitVal, float rIn, float rOut, uint16_t bg, uint16_t tipColor) {
  (void)tipColor;
  const float v = constrain(unitVal, 0.0f, gpsScaleMax());
  arcSweepC(g, 0, 0, 0, gpsArcAngle(0.0f), gpsArcAngle(v), rIn, rOut, bg);   // proposta C
}
static void arcPolarSel(int arcSel, float r, float deg, int &x, int &y) {
  if (arcSel == 0) gpsArcPt(r, deg, 0, x, y);
  else { gsPolar(r, deg, 0, x, y); if (arcSel == 2) x = 360 - x; }
}
static void gpsArcBand(lgfx::LGFXBase &g, float rIn, float rOut, float v0, float v1, uint16_t color) {
  if (v1 <= v0) return;
  fillArcAA(g, 179.5f, 179.5f, rIn, rOut, gpsArcAngle(v1), gpsArcAngle(v0), false, color, themeBg());   // bordi sfumati
}
__attribute__((unused)) static void gpsArcTriangle(lgfx::LGFXBase &g, float deg, float rTip, float rBase, float half, uint16_t color) {
  int x0, y0, x1, y1, x2, y2;
  gpsArcPt(rTip, deg, 0, x0, y0);
  gpsArcPt(rBase, deg, half, x1, y1);
  gpsArcPt(rBase, deg, -half, x2, y2);
  (void)color;
  drawArcPointerTri(g, x0, y0, x1, y1, x2, y2);
}
// Scrive "<num> <unit>" centrato: numero nel colore numColor, unita' in ciano (font UI gia' caricato).
static void gpsDrawValueUnit(lgfx::LGFXBase &g, const char *num, const char *unit, int y, uint16_t numColor, uint16_t bg) {
  const int wN = g.textWidth(num), wS = g.textWidth(" "), wU = g.textWidth(unit);
  int x = 180 - (wN + wS + wU) / 2;
  g.setTextDatum(ML_DATUM);
  g.setTextColor(numColor, bg);
  g.drawString(num, x, y);
  x += wN + wS;
  g.setTextColor(themeColor(TFT_CYAN), bg);
  g.drawString(unit, x, y);
}

// Lancetta da contachilometri: base larga 16 px sul cerchio (invisibile) attorno al numero, si stringe
// fino a una punta che sfiora il bordo interno dell'arco. Contorno bianco di 2 px.
__attribute__((unused)) static void gpsDrawSpeedNeedle(lgfx::LGFXBase &g, float deg, float rBase, float rTip, uint16_t color) {
  int bx1, by1, bx2, by2, mx1, my1, mx2, my2, tx, ty;
  const float rMid = rBase + (rTip - rBase) * 0.25f;
  gpsArcPt(rBase, deg, 8.0f, bx1, by1);    // base larga 16 px
  gpsArcPt(rBase, deg, -8.0f, bx2, by2);
  gpsArcPt(rMid, deg, 5.0f, mx1, my1);
  gpsArcPt(rMid, deg, -5.0f, mx2, my2);
  gpsArcPt(rTip, deg, 0.0f, tx, ty);
  g.fillTriangle(bx1, by1, mx1, my1, mx2, my2, color);
  g.fillTriangle(bx1, by1, mx2, my2, bx2, by2, color);
  g.fillTriangle(mx1, my1, tx, ty, mx2, my2, color);
  // Contorno bianco di 2 px
  const uint16_t edge = themeColor(TFT_WHITE);
  drawThickLineOn(g, bx1, by1, mx1, my1, edge, 2);
  drawThickLineOn(g, mx1, my1, tx, ty, edge, 2);
  drawThickLineOn(g, tx, ty, mx2, my2, edge, 2);
  drawThickLineOn(g, mx2, my2, bx2, by2, edge, 2);
  drawThickLineOn(g, bx2, by2, bx1, by1, edge, 2);
}
// Raggio del cerchio (invisibile, centro del quadrante) da cui parte la lancetta: contiene "200" scritto con il numero grande,
// che e' centrato a y = 160. Calcolato una volta sola con il carattere vero.
static int gpsHubRadius(lgfx::LGFXBase &g) {
  static int r = 0;
  if (r > 0) return r;
  const bool l = vlwSet(g, VarioNumbers80);
  if (l) g.setTextSize(1.14f, 1.14f);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b); g.setTextSize(1.2f, 1.2f); }
  const float w = (float)g.textWidth("200");
  const float h = (float)g.fontHeight() * 0.75f;   // altezza delle cifre (la riga del carattere e' piu' alta)
  if (l) vlwRelease(g);
  r = constrain((int)lroundf(hypotf(w * 0.5f, 20.0f + h * 0.5f)) + 4, 70, 112);   // 112: resta dentro i numeri della scala
  return r;
}

static float gpsNeedleKmh = 0.0f;   // valore disegnato dalla lancetta (insegue quello vero)
static void renderGpsSpeedGauge(bool valid, float speedKmh) {
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  const uint16_t bg = themeBg();
  if (spr) navigationSprite.fillSprite(bg); else lcd.fillScreen(bg);
  const float maxU = gpsScaleMax();
  const float lowU = min(gpsToUnit(LOW_GS_KMH), maxU), marU = min(gpsToUnit(GS_LOW_MARGIN_KMH), maxU);
  const float yelU = max(gpsToUnit((float)gsYellowKmh), marU), redU = max(gpsToUnit((float)gsRedKmh), yelU);
  const uint16_t cLow = LOW_GS_COLOR;   // rettangolo 65-85 sempre arancione
  const uint16_t cG = arcSafeColor(), cY = themeVivid(TFT_YELLOW), cR = themeVivid(TFT_RED);   // fascia normale verde
  const uint16_t cOrange = themeVivid(TFT_ORANGE), cWhite = themeColor(TFT_WHITE);

  // Scia della velocita' (al posto della lancetta), sotto tacche e numeri
  const float needleBase = max((float)gpsHubRadius(g) + 1.0f, 108.0f);
  if (valid) gpsDrawSpeedSweep(g, gpsToUnit(gpsNeedleKmh), needleBase, 161.0f, bg, cOrange);
  // Fasce colorate (0-65 km/h: nessuna fascia, tacche allungate e arco bianco esterno 4 px)
  const float RO = ARC_R_OUT;   // bordo esterno: filo nero costante dal vetro
  gpsArcBand(g, RO - 4.0f, RO, 0, lowU, cWhite);
  if (marU > lowU && gsRectVisible()) {   // 65 - 85 km/h: solo contorno (2.5 px), staccato di ~3 px dai separatori neri
    const float upd = maxU / 270.0f;                 // unita' di velocita' per grado
    const float IN = 1.0f * upd, CAP = 0.85f * upd;  // ~3 px e ~2.5 px a r 170
    if (marU - lowU > 2.0f * IN + 2.0f * CAP) {
      gpsArcBand(g, 162.0f, 164.5f, lowU + IN, marU - IN, cLow);
      gpsArcBand(g, RO - 2.5f, RO, lowU + IN, marU - IN, cLow);
      gpsArcBand(g, 162.0f, RO, lowU + IN, lowU + IN + CAP, cLow);
      gpsArcBand(g, 162.0f, RO, marU - IN - CAP, marU - IN, cLow);
    } else gpsArcBand(g, 162.0f, RO, lowU, marU, cLow);
  }
  gpsArcBand(g, 162.0f, RO, marU, min(yelU, maxU), cG);
  gpsArcBand(g, 162.0f, RO, min(yelU, maxU), min(redU, maxU), cY);
  gpsArcBand(g, 162.0f, RO, min(redU, maxU), maxU, cR);
  // Trattini neri (4 px) ai bordi delle fasce: zona lenta, margine, giallo e rosso
  for (int e = 0; e < 4; ++e) {
    const float ev = e == 0 ? lowU : (e == 1 ? marU : (e == 2 ? yelU : redU));
    if (ev <= 0.0f || ev >= maxU) continue;
    int x0, y0, x1, y1;
    gpsArcPt(161, gpsArcAngle(ev), 0, x0, y0);
    gpsArcPt(180, gpsArcAngle(ev), 0, x1, y1);
    drawThickLineOn(g, x0, y0, x1, y1, TFT_BLACK, 4);   // trattino nero 4 px
  }
  // Soglia in regolazione: evidenziata con una barretta bianca spessa
  if (gpsSpeedDragTarget != 0) {
    const float b = gpsSpeedDragTarget == 1 ? yelU : redU;
    int x0, y0, x1, y1;
    gpsArcPt(156, gpsArcAngle(b), 0, x0, y0);
    gpsArcPt(180, gpsArcAngle(b), 0, x1, y1);
    g.drawWideLine(x0, y0, x1, y1, 2.5f, cWhite);
  }
  // Tacche e scritte (passo piu' largo quando il fondo scala cresce)
  const int tickStep = gpsTickStepFor(gpsSpeedShowKnots, maxU);
  const int majorStep = gpsLabelStepFor(gpsSpeedShowKnots, maxU);
  for (int k = 0; k <= (int)maxU; k += tickStep) {
    int x0, y0, x1, y1;
    gpsArcPt(k % majorStep == 0 ? 142.0f : 151.0f, gpsArcAngle((float)k), 0, x0, y0);   // come ALTIMETER
    gpsArcPt((float)k < lowU - 0.01f ? ARC_TICK_R : 161.0f, gpsArcAngle((float)k), 0, x1, y1);   // 0-65: fino all'arco bianco
    drawThickLineOn(g, x0, y0, x1, y1, cWhite, 2);   // stesso spessore delle tacche ALTIMETER
  }
  for (float k = tickStep / 2.0f; !gsScaleIs150(gpsSpeedShowKnots, maxU) && k < lowU - 0.01f; k += tickStep) {   // 0-65: tacche intermedie corte (non in 0-150)
    int x0, y0, x1, y1;
    gpsArcPt(167.0f, gpsArcAngle(k), 0, x0, y0);
    gpsArcPt(ARC_TICK_R, gpsArcAngle(k), 0, x1, y1);
    g.drawWideLine(x0, y0, x1, y1, 0.8f, cWhite);
  }
  g.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  {   // numeri dell'arco +30% (un po' piu' piccoli sulle scale grandi, che ne hanno di piu')
    const float ns = (maxU > (gpsSpeedShowKnots ? 110.5f : 200.5f) ? 1.0f : 1.17f) * THEME_TEXT_SCALE;
    g.setTextSize(ns, ns);
  }
  g.setTextDatum(MC_DATUM);
  for (int k = 0; k <= (int)maxU; k += majorStep) {
    int lx, ly;
    const float la = gpsArcAngle((float)k);
    gpsArcPt(la > 30.0f && la < 150.0f ? 129.0f : 122.0f, la, 0, lx, ly);   // in alto piu' fuori: lontano dalla scritta GPS SPEED
    char b[6];
    snprintf(b, sizeof(b), "%d", k);
    // sopra la scia lo sfondo non e' piu' uniforme: testo trasparente; i numeri gia' superati piu' chiari
    const bool passed = valid && (float)k <= gpsToUnit(gpsNeedleKmh);
    g.setTextColor(passed ? themeColor(TFT_WHITE) : themeColor(TFT_LIGHTGREY));
    g.drawString(b, lx, ly);
  }
  // Massima del volo: triangolino arancione fuori dalla fascia
  const float maxSpeedU = gpsToUnit(maxGpsSpeedKmh);
  (void)maxSpeedU;   // il triangolino della massima sull'arco non c'e' piu'
  // (la lancetta e' sostituita dalla scia disegnata sotto tacche e numeri)

  // Testi centrali
  const char *mainUnit = gpsSpeedShowKnots ? "kt" : "km/h";
  const char *otherUnit = gpsSpeedShowKnots ? "km/h" : "kt";
  const int mainVal = (int)lroundf(gpsToUnit(speedKmh));
  const int otherVal = (int)lroundf(gpsSpeedShowKnots ? speedKmh : speedKmh / 1.852f);
  char buf[24];
  bool loaded = vlwSet(g, UiFont32);
  {
    // Titolo GND SPEED su una riga, piu' piccolo (0.62), abbassato finche' gli angoli in alto sfiorano
    // l'arco interno (raggio needleBase) con ~1,5 px di aria. LOW GS (0.75) segue la stessa regola.
    const float k = lowGsNow ? 0.75f : 0.62f;
    if (loaded) g.setTextSize(k, k);
    else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(1.1f * k / 0.75f, 1.1f * k / 0.75f); }
    g.setTextDatum(MC_DATUM);
    const char *title = lowGsNow ? "LOW GS" : "GND SPEED";
    const float hw = 0.5f * (float)g.textWidth(title);
    const float capH = 0.70f * (float)g.fontHeight();
    const float rr = max(needleBase, hw + 1.0f);
    const int ty = (int)lroundf(180.0f - sqrtf(rr * rr - hw * hw) + 1.5f + capH * 0.5f) + 8;   // +8 px: staccata dall'arco
    if (lowGsNow) {   // LOW GS arancione lampeggiante al posto del titolo
      g.setTextColor(LOW_GS_COLOR, bg);
      if (lowGsBlinkOn()) g.drawString(title, 180, ty);
    } else {
      g.setTextColor(themeColor(TFT_CYAN), bg);
      g.drawString(title, 180, ty);
    }
  }
  if (loaded) vlwRelease(g);

  // Valore grande: colore della fascia (solo tema a colori)
  loaded = vlwSet(g, VarioNumbers80);
  if (loaded) g.setTextSize(1.14f, 1.14f);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b); g.setTextSize(1.2f, 1.2f); }
  if (valid) snprintf(buf, sizeof(buf), "%d", mainVal); else snprintf(buf, sizeof(buf), "--");
  g.setTextDatum(MC_DATUM);
  // Tema a colori: verde/giallo/rosso come la fascia in cui cade la velocita'; altri temi: colore del testo del tema
  const uint16_t mainColor = lowGsNow ? LOW_GS_COLOR : ((displayTheme == THEME_COLOR) ? gsSpeedColor(speedKmh) : cWhite);
  g.setTextColor(valid ? mainColor : themeColor(TFT_DARKGREY));   // sfondo trasparente (niente riquadro)
  g.drawString(buf, 180, 165);   // valore 5 px piu' giu'
  if (loaded) vlwRelease(g);

  loaded = vlwSet(g, UiFont32);
  if (loaded) g.setTextSize(0.85f, 0.85f);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(1.2f, 1.2f); }
  g.setTextDatum(MC_DATUM);
  g.setTextColor(themeColor(TFT_CYAN), bg);
  g.drawString(mainUnit, 180, 218);
  if (gpsSpeedDragTarget != 0) {
    // Durante la regolazione: valore della soglia nel suo colore
    const int b = (int)lroundf(gpsSpeedDragTarget == 1 ? yelU : redU);
    snprintf(buf, sizeof(buf), gpsSpeedDragTarget == 1 ? "YELLOW %d" : "RED %d", b);
    g.setTextColor(gpsSpeedDragTarget == 1 ? cY : cR, bg);
    g.drawString(buf, 180, 256);
  } else {
    if (valid) snprintf(buf, sizeof(buf), "%d", otherVal); else snprintf(buf, sizeof(buf), "--");
    gpsDrawValueUnit(g, buf, otherUnit, 256, cWhite, bg);
  }
  if (loaded) vlwRelease(g);

  // MAX in basso, nello spazio aperto dell'arco
  g.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  g.setTextSize(1.5f * THEME_TEXT_SCALE, 1.5f * THEME_TEXT_SCALE);   // +50%
  snprintf(buf, sizeof(buf), "MAX %d %s", (int)lroundf(maxSpeedU), mainUnit);
  g.setTextDatum(MC_DATUM);
  g.setTextColor(cOrange, bg);
  g.drawString(buf, 180, 308);   // piu' in alto: resta dentro il bordo tondo
  g.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);

  // lancetta per ultima: in primo piano sopra fasce, tacche e numeri
  if (valid) arcNeedleC(g, 0, 0, 0, gpsArcAngle(constrain(gpsToUnit(gpsNeedleKmh), 0.0f, gpsScaleMax())), needleBase, 161.0f, bg);
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

void drawGpsGroundSpeedScreen() {
  gpsSpeedLastKey = 0xFFFFFFFFu;
  if (!ensureNavigationSprite()) lcd.fillScreen(themeBg());
  updateGpsGroundSpeed();
}

void updateGpsGroundSpeed() {
  if (currentScreen != GPS_SPEED_SCREEN) return;
  float speedKmh = 0.0f;
  bool valid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    speedKmh = gps_ground_speed_kmh;
    valid = gpsDataValid && isfinite(speedKmh);
    xSemaphoreGive(sensorDataMutex);
  }
  if (!valid) speedKmh = 0.0f;
  const bool rangeChanged = gsUpdateAutoRange(speedKmh, valid);   // fondo scala automatico (come ALTIMETER)
  // Ridisegno solo se cambia qualcosa di visibile (valori interi nell'unita' mostrata).
  static unsigned long ndlMs = 0, ndlDrawMs = 0;
  needleFollow(gpsNeedleKmh, ndlMs, speedKmh, valid);   // lancetta che scorre verso la velocita' vera
  const int shown = valid ? (int)lroundf(gpsToUnit(gpsNeedleKmh) * 4.0f) : -1;   // quarti di unita': lancetta fluida
  static int lastNum = -9999;
  const int num = valid ? (int)lroundf(gpsToUnit(speedKmh)) : -1;   // il numero grande resta quello vero
  const int shownMax = (int)lroundf(gpsToUnit(maxGpsSpeedKmh));
  const uint32_t key = ((uint32_t)(shown & 0x7FF)) | ((uint32_t)(shownMax & 0x3FF) << 11) |
                       ((uint32_t)gpsSpeedShowKnots << 21) | ((uint32_t)(gpsSpeedDragTarget & 3) << 22) |
                       ((uint32_t)((gsYellowKmh * 7 + gsRedKmh * 13) & 0xFF) << 24);
  const uint8_t lowKey = (lowGsNow ? (lowGsBlinkOn() ? 2 : 1) : 0) | (lowGsArmed ? 4 : 0) |
                         (gsRectBlinkNow ? (lowGsBlinkOn() ? 16 : 8) : 0);   // LOW GS, lampeggio, zona lenta, rettangolo
  static uint8_t lastLowKey = 0;
  if (key == gpsSpeedLastKey && !rangeChanged && lowKey == lastLowKey && num == lastNum) return;
  if (!rangeChanged && lowKey == lastLowKey && num == lastNum && millis() - ndlDrawMs < 25UL) return;   // max 40 al secondo
  lastLowKey = lowKey;
  lastNum = num;
  gpsSpeedLastKey = key;
  ndlDrawMs = millis();
  renderGpsSpeedGauge(valid, speedKmh);
}

// Valore della scala (unita' mostrata) sotto il dito; false se il punto non e' sull'arco.
static bool gpsArcValueAt(int x, int y, float &unitVal) {
  const float dx = (float)x - 180.0f, dy = 180.0f - (float)y;
  const float r = sqrtf(dx * dx + dy * dy);
  if (r < 110.0f) return false;
  float d = 225.0f - atan2f(dy, dx) * RAD_TO_DEG;
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  if (d > 270.0f) d = (d < 315.0f) ? 270.0f : 0.0f;   // nello spazio in basso: estremo piu' vicino
  unitVal = d / 270.0f * gpsScaleMax();
  return true;
}

// Chiamata mentre il dito e' appoggiato: swipe sull'arco = sposta la soglia piu' vicina al punto iniziale.
void gpsSpeedTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != GPS_SPEED_SCREEN) return;
  if (touchId != gpsSpeedDragTouchId) {
    gpsSpeedDragTouchId = touchId;
    gpsSpeedDragMoved = false;
    gpsSpeedDragTarget = 0;
    float v0;
    if (gpsArcValueAt(startX, startY, v0)) {
      const float dY = fabsf(v0 - gpsToUnit((float)gsYellowKmh));
      const float dR = fabsf(v0 - gpsToUnit((float)gsRedKmh));
      gpsSpeedDragTarget = (dY <= dR) ? 1 : 2;
    }
    if (realFlightActive() || arcBlockEnabled) gpsSpeedDragTarget = 0;   // in volo reale o con ARCH BLOCK le soglie non si cambiano
    return;
  }
  if (gpsSpeedDragTarget == 0) return;
  if (!gpsSpeedDragMoved && abs(x - startX) < 8 && abs(y - startY) < 8) return;
  if (!gpsSpeedDragMoved && arcDragIsRadial(startX, startY, x, y)) { gpsSpeedDragTarget = 0; gpsSpeedLastKey = 0xFFFFFFFFu; return; }   // e' uno swipe
  gpsSpeedDragMoved = true;
  float v;
  if (!gpsArcValueAt(x, y, v)) return;
  const float step = 5.0f;   // 5 km/h oppure 5 nodi
  v = roundf(v / step) * step;
  int kmh = (int)lroundf(gpsSpeedShowKnots ? v * 1.852f : v);
  if (gpsSpeedDragTarget == 1) {   // la gialla puo' spingere avanti la rossa
    gsYellowKmh = constrain(kmh, GS_YELLOW_MIN_KMH, GS_YELLOW_MAX_KMH);
    if (gsRedKmh < gsYellowKmh + 10) gsRedKmh = gsYellowKmh + 10;
  } else gsRedKmh = constrain(kmh, gsYellowKmh + 10, GS_RED_MAX_KMH);
  if (gsUpdateAutoRange(0.0f, false)) gsScaleNeedsErase = true;   // gialla a 200 km/h: scala 0-300 subito
  if (millis() - gpsSpeedLastDragDrawMs >= 50) {
    gpsSpeedLastDragDrawMs = millis();
    updateGpsGroundSpeed();
  }
}

// Chiamata al rilascio. true = tocco gestito qui (niente cambio pagina).
bool gpsSpeedTouchRelease(bool tapValid, int startX, int startY) {
  if (currentScreen != GPS_SPEED_SCREEN) return false;
  const bool wasDrag = gpsSpeedDragTarget != 0 && gpsSpeedDragMoved;
  const bool armed = gpsSpeedDragTarget != 0;
  gpsSpeedDragTarget = 0;
  gpsSpeedDragTouchId = 0;
  gpsSpeedDragMoved = false;
  if (wasDrag) {
    preferences.begin("altimeter", false);
    preferences.putInt("gs_yel", gsYellowKmh);
    preferences.putInt("gs_red", gsRedKmh);
    preferences.end();
    bleQueueArc();   // slave: le soglie vanno al master, che le applica e le rinvia (vale anche per ALTIMETER)
    updateGpsGroundSpeed();
    playTone(1200, 50);
    return true;
  }
  if (armed) updateGpsGroundSpeed();   // toglie l'evidenziazione
  const int dx = startX - 180, dy = startY - 180;
  if (tapValid && dx * dx + dy * dy <= 105 * 105) {
    gpsSpeedShowKnots = !gpsSpeedShowKnots;
    preferences.begin("altimeter", false);
    preferences.putBool("gs_kt", gpsSpeedShowKnots);
    preferences.end();
    bleQueuePrefs();   // slave: km/h <-> nodi anche sul master
    updateGpsGroundSpeed();
    playTone(1200, 50);
    return true;
  }
  return armed;   // tocco iniziato sull'arco: non cambia pagina
}

// Soglie giallo/rosso dell'arco velocita' ricevute via BLE (master <-> slave). ALTIMETER e GPS SPEED usano
// le stesse variabili gsYellowKmh / gsRedKmh, quindi una modifica su una pagina vale anche per l'altra.
void arcApplyShared(int yelKmh, int redKmh) {
  if (altGsDragTarget != 0 || gpsSpeedDragTarget != 0) return;   // sto trascinando io: non sovrascrivere
  const int y = constrain(yelKmh, GS_YELLOW_MIN_KMH, GS_YELLOW_MAX_KMH);
  const int r = constrain(redKmh, y + 10, GS_RED_MAX_KMH);
  if (y == gsYellowKmh && r == gsRedKmh) return;
  gsYellowKmh = y;
  gsRedKmh = r;
  preferences.begin("altimeter", false);
  preferences.putInt("gs_yel", gsYellowKmh);
  preferences.putInt("gs_red", gsRedKmh);
  preferences.end();
  if (currentScreen == MAIN_ALT_SCREEN) updateMainGsGauge(true);
  else if (currentScreen == GPS_SPEED_SCREEN) updateGpsGroundSpeed();   // la chiave include le soglie: ridisegna
}

void drawGpsClockScreen() {
  lcd.fillScreen(themeBg());
  lastPrintedClockKey = 0xFFFFFFFFUL;
  gpsClockHasValidTime = false;
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.5f) * THEME_TEXT_SCALE, (1.5f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("GPS CLOCK", 180, 55, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);
  updateGpsClock();
}

void calculateSunTimes(double lat, double lon, int day, int month, int year, double utcOffset) {   // utcOffset in ore (anche 5.5, 5.75)
  int n1 = (int)(275 * month / 9);
  int n2 = (int)((month + 9) / 12);
  int n3 = (1 + (year - 4 * (year / 4) + 2) / 3);
  int dayOfYear = n1 - (n2 * n3) + day - 30;
  double lngHour = lon / 15.0;
  auto eventTime = [&](bool rise) -> double {
    double t = dayOfYear + ((rise ? 6.0 : 18.0) - lngHour) / 24.0;
    double M = (0.9856 * t) - 3.289;
    double L = M + (1.916 * sin(M * DEG_TO_RAD)) + (0.020 * sin(2 * M * DEG_TO_RAD)) + 282.634;
    while (L < 0) L += 360; while (L >= 360) L -= 360;
    double RA = atan(0.91764 * tan(L * DEG_TO_RAD)) / DEG_TO_RAD;
    while (RA < 0) RA += 360; while (RA >= 360) RA -= 360;
    RA += 90.0 * ((int)(L / 90.0) - (int)(RA / 90.0));
    double sinDec = 0.39782 * sin(L * DEG_TO_RAD);
    double cosDec = cos(asin(sinDec));
    double cosH = (cos(90.833 * DEG_TO_RAD) - sin(lat * DEG_TO_RAD) * sinDec) /
                  (cos(lat * DEG_TO_RAD) * cosDec);
    if (cosH > 1.0 || cosH < -1.0) return NAN;
    double H = (rise ? 360.0 - acos(cosH) / DEG_TO_RAD : acos(cosH) / DEG_TO_RAD) / 15.0;
    double T = H + RA / 15.0 - (0.06571 * t) - 6.622;
    double utc = T - lngHour;
    double local = utc + utcOffset;
    while (local < 0) local += 24; while (local >= 24) local -= 24;
    return local;
  };
  double rise = eventTime(true), set = eventTime(false);
  sunTimesValid = isfinite(rise) && isfinite(set);
  if (sunTimesValid) {
    auto formatSunTime = [](double value, char *out, size_t outSize) {
      int hour = (int)value;
      int minute = (int)round((value - hour) * 60.0);
      if (minute >= 60) {
        minute = 0;
        hour = (hour + 1) % 24;
      }
      snprintf(out, outSize, "%02d:%02d", hour, minute);
    };
    formatSunTime(rise, sunriseText, sizeof(sunriseText));
    formatSunTime(set, sunsetText, sizeof(sunsetText));
  } else { strcpy(sunriseText, "--:--"); strcpy(sunsetText, "--:--"); }
}

// offset = scostamento dall'UTC in MINUTI (fuso automatico, ora legale compresa)
void updateSunTimes(double lat, double lon, int day, int month, int year, int offset) {
  int dayKey = year * 10000 + month * 100 + day;
  if (sunTimesValid && dayKey == lastSunDayKey && offset == lastSunUtcOffset &&
      fabs(lat - lastSunLat) < 0.01 && fabs(lon - lastSunLon) < 0.01) return;
  calculateSunTimes(lat, lon, day, month, year, offset / 60.0);
  lastSunDayKey = dayKey; lastSunUtcOffset = offset;
  lastSunLat = lat; lastSunLon = lon;
}

void updateGpsClock() {
  if (currentScreen != GPS_CLOCK_SCREEN) return;
  int localHour, minute, second, localDay, localMonth, localYear;
  bool valid = getLocalClock(localHour, minute, second, localDay, localMonth, localYear);
  if (!valid) {
    if (gpsClockHasValidTime) return;
    if (lastPrintedClockKey == 0xFFFFFFFEUL) return;
    lcd.fillRect(15, 90, 330, 155, themeBg());
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    setThemeTextColor(TFT_RED, TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString("GPS TIME", 180, 145);
    lcd.drawString("UNAVAILABLE", 180, 185);
    lastPrintedClockKey = 0xFFFFFFFEUL;
    return;
  }
  uint32_t clockKey = (uint32_t)((((localYear - 2000) * 12 + localMonth) * 31 + localDay) * 86400UL + localHour * 3600UL + minute * 60UL + second);
  if (clockKey == lastPrintedClockKey) return;
  bool firstValidFix = !gpsClockHasValidTime;
  lastPrintedClockKey = clockKey;
  gpsClockHasValidTime = true;
  char timeBuf[16];
  char dayBuf[4], monthBuf[4], yearBuf[6];
  char zoneBuf[48];
  const int offset = (int)(tzAutoOffsetSec / 60);   // minuti (calcolato da getLocalClock qui sopra)
  snprintf(timeBuf, sizeof(timeBuf), "%02d:%02d:%02d", localHour, minute, second);
  snprintf(dayBuf, sizeof(dayBuf), "%02d", localDay);
  snprintf(monthBuf, sizeof(monthBuf), "%02d", localMonth);
  snprintf(yearBuf, sizeof(yearBuf), "%04d", localYear);
  const bool gpsTimeFresh = (millis() - softwareClockLastGpsSyncMillis) <= GPS_CLOCK_FRESH_MS;
  // Sigla della zona (es. "CEST") e scostamento; con sigla numerica solo "UTC+3"
  char offBuf[24];
  const int absMin = abs(offset);
  if (absMin % 60) snprintf(offBuf, sizeof(offBuf), "UTC%c%d:%02d", offset < 0 ? '-' : '+', absMin / 60, absMin % 60);
  else snprintf(offBuf, sizeof(offBuf), "UTC%c%d", offset < 0 ? '-' : '+', absMin / 60);
  if (strncmp(tzAutoAbbr, "UTC", 3) == 0) snprintf(zoneBuf, sizeof(zoneBuf), "%s %s", offBuf, gpsTimeFresh ? "GPS" : "HOLDOVER");
  else snprintf(zoneBuf, sizeof(zoneBuf), "%s %s %s", tzAutoAbbr, offBuf, gpsTimeFresh ? "GPS" : "HOLDOVER");
  double lat = gps_latitude, lon = gps_longitude;
  updateSunTimes(lat, lon, localDay, localMonth, localYear, offset);
  // Riquadri colorati: ora su fondo blu scuro con doppia cornice blu, data con cornice verde,
  // tramonto in un riquadro arancio con l'icona del sole. Nei temi a due colori: cornici del tema, fondo del tema.
  const bool clkColor = displayTheme == THEME_COLOR;
  const uint16_t timeBoxBg = clkColor ? lgfx::color565(10, 30, 80) : themeBg();
  const uint16_t sunBoxBg = clkColor ? lgfx::color565(58, 34, 0) : themeBg();
  const uint16_t sunCol = clkColor ? lgfx::color565(255, 144, 32) : themeColor(TFT_YELLOW);
  if (firstValidFix) {
    lcd.fillRect(15, 80, 330, 240, themeBg());
    const uint16_t blueFrame = clkColor ? lgfx::color565(47, 125, 255) : themeColor(TFT_CYAN);
    lcd.fillRoundRect(40, 83, 280, 72, 14, timeBoxBg);   // centrato sulle cifre (centro y 119)
    lcd.drawRoundRect(40, 83, 280, 72, 14, blueFrame);
    lcd.drawRoundRect(41, 84, 278, 70, 13, blueFrame);
    lcd.drawRoundRect(28, 165, 304, 52, 10, clkColor ? lgfx::color565(34, 212, 106) : themeColor(TFT_GREEN));
    lcd.fillRoundRect(70, 262, 220, 36, 10, sunBoxBg);
    lcd.drawRoundRect(70, 262, 220, 36, 10, sunCol);
    lcd.fillCircle(94, 280, 4, sunCol);   // sole
    for (int i = 0; i < 8; ++i) {
      const float a = i * 45.0f * DEG_TO_RAD;
      lcd.drawWideLine(94.0f + 7.0f * cosf(a), 280.0f + 7.0f * sinf(a), 94.0f + 12.0f * cosf(a), 280.0f + 12.0f * sinf(a), 1.0f, sunCol);
    }
  }

  drawTimerGroup(timeBuf, 130, TFT_WHITE, 1.25f * THEME_TEXT_SCALE, timeBoxBg);

  lcd.setTextDatum(MC_DATUM);
  const bool dateVlw = vlwSet(lcd, ClockFont48);
  if (dateVlw) {
    lcd.setTextSize(1.0f, 1.0f);
  } else {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  }
  lcd.setTextColor(clkColor ? lgfx::color565(34, 212, 106) : themeColor(TFT_GREEN), themeBg());   // data verde
  // Con il font VLW il centro cifre e' circa 9 px sopra la y usata col vecchio font.
  const int dateY = dateVlw ? 200 - 9 - (int)lroundf(ClockFont48_DIGIT_CENTER_DY) : 200;
  const int dateGap = 7;
  const int dayWidth = lcd.textWidth("00");
  const int slashWidth = lcd.textWidth("/");
  const int monthWidth = lcd.textWidth("00");
  const int yearWidth = lcd.textWidth("0000");
  const int dateWidth = dayWidth + slashWidth + monthWidth + slashWidth + yearWidth + 4 * dateGap;
  int dateX = (360 - dateWidth) / 2;
  const int dayCenter = dateX + dayWidth / 2;
  dateX += dayWidth + dateGap;
  const int firstSlashCenter = dateX + slashWidth / 2;
  dateX += slashWidth + dateGap;
  const int monthCenter = dateX + monthWidth / 2;
  dateX += monthWidth + dateGap;
  const int secondSlashCenter = dateX + slashWidth / 2;
  dateX += slashWidth + dateGap;
  const int yearCenter = dateX + yearWidth / 2;
  lcd.drawString(dayBuf, dayCenter, dateY);
  lcd.drawString("/", firstSlashCenter, dateY);
  lcd.drawString(monthBuf, monthCenter, dateY);
  lcd.drawString("/", secondSlashCenter, dateY);
  lcd.drawString(yearBuf, yearCenter, dateY);
  if (dateVlw) vlwRelease(lcd);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString(zoneBuf, 180, 245);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.setTextColor(sunCol, sunBoxBg);
  char sunBuf[32];
  snprintf(sunBuf, sizeof(sunBuf), "SUNSET %s", sunsetText);
  lcd.drawString(sunBuf, 196, 280);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString(tzAutoIsDst ? "AUTO ZONE (DST)" : "AUTO ZONE", 180, 315);
}

// ========== SCHERMATA GPS INFO (con redraw senza sfarfallio) ==========
void drawGpsInfoScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.4f) * THEME_TEXT_SCALE, (1.4f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  drawUiVlw("GPS INFO", 180, 42, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);
  lcd.drawLine(20, 66, 340, 66, themeColor(TFT_DARKGREY));

  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.setTextDatum(ML_DATUM);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("FIX:",     40, 95);
  lcd.drawString("QUALITY:", 40, 125);
  lcd.drawString("SATS:",    40, 155);

  gpsInfoLastFix[0]     = '\0';
  gpsInfoLastFixColor   = 0xFFFF;
  gpsInfoLastQuality[0] = '\0';
  gpsInfoLastSats       = -1;
  gpsInfoLastSatsValid  = false;
  gpsInfoLastHasGps     = false;
  gpsInfoLastHasGlonass = false;
  gpsInfoLastHasGalileo = false;
  gpsInfoLastHasBds     = false;
  gpsInfoLastHasQzss    = false;
  gpsInfoConstInit      = false;
  gpsInfoLastUtcH       = -1;
  gpsInfoLastUtcM       = -1;
  gpsInfoLastUtcS       = -1;
  gpsInfoLastTimeValid  = false;
  bleUiLastKey = -1;

  updateGpsInfoScreen();
}

// Riga di stato BLE in fondo alla schermata GPS INFO. Un tocco sulla riga cambia ruolo: OFF > MASTER > SLAVE.
static void drawBleStatusLine() {
  const int key = (int)bleRole * 10 + (int)bleState + (displayTheme == THEME_COLOR ? 100 : 0);
  if (key == bleUiLastKey) return;
  bleUiLastKey = key;
  const char *roleTxt = bleRole == BLE_ROLE_MASTER ? "MASTER" : (bleRole == BLE_ROLE_SLAVE ? "SLAVE" : "OFF");
  const char *stTxt = "";
  uint16_t col = TFT_DARKGREY;
  switch (bleState) {
    case BLE_ST_WAIT:   stTxt = (bleRole == BLE_ROLE_SLAVE) ? " SCAN" : " WAIT"; col = TFT_YELLOW; break;
    case BLE_ST_LINKED: stTxt = " LINK"; col = TFT_GREEN; break;
    case BLE_ST_ERROR:  stTxt = " ERR";  col = TFT_RED;   break;
    case BLE_ST_PAUSED: stTxt = " WIFI"; col = TFT_YELLOW; break;
    default: break;
  }
  char buf[32];
  snprintf(buf, sizeof(buf), "BLE %s%s", roleTxt, stTxt);
  const uint16_t bg = themeBg();
  lcd.fillRect(75, 308, 210, 28, bg);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.setTextDatum(MC_DATUM);
  if (displayTheme == THEME_COLOR) lcd.setTextColor(col, bg);
  else lcd.setTextColor(TFT_BLACK, bg);
  lcd.drawString(buf, 180, 322);
}

void updateGpsInfoScreen() {
  if (currentScreen != GPS_INFO_SCREEN) return;

  int fixType = 0, fixQuality = 0, sats = 0;
  bool valid = false;
  int utcH = 0, utcM = 0, utcS = 0;
  int day = 0;
  bool hasGps = false, hasGlonass = false, hasGalileo = false, hasBds = false, hasQzss = false;

  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    fixType    = gps_fix_type;
    fixQuality = gps_fix_quality;
    sats       = gps_satellites;
    valid      = gpsDataValid;
    utcH       = gps_utc_hours;
    utcM       = gps_utc_minutes;
    utcS       = gps_utc_seconds;
    day        = gps_utc_day;
    unsigned long nowMs = millis();
    hasGps     = gps_has_gps     && (nowMs - gps_last_gsv_gps     < GSV_TIMEOUT_MS);
    hasGlonass = gps_has_glonass && (nowMs - gps_last_gsv_glonass < GSV_TIMEOUT_MS);
    hasGalileo = gps_has_galileo && (nowMs - gps_last_gsv_galileo < GSV_TIMEOUT_MS);
    hasBds     = gps_has_bds     && (nowMs - gps_last_gsv_bds     < GSV_TIMEOUT_MS);
    hasQzss    = gps_has_qzss    && (nowMs - gps_last_gsv_qzss    < GSV_TIMEOUT_MS);
    xSemaphoreGive(sensorDataMutex);
  }

  const uint16_t bgColor     = themeBg();
  const uint16_t valueRightX = GPS_INFO_VALUE_RIGHT_X;

  const char* fixLabel = "NO FIX";
  uint16_t fixColor = TFT_RED;
  if (valid) {
    if (fixType == 3)      { fixLabel = "3D FIX"; fixColor = TFT_GREEN;  }
    else if (fixType == 2) { fixLabel = "2D FIX"; fixColor = TFT_YELLOW; }
    else if (fixType == 1) { fixLabel = "NO FIX"; fixColor = TFT_RED;    }
    else if (fixQuality > 0) { fixLabel = "FIX";  fixColor = TFT_GREEN;  }
  }
  if (strcmp(fixLabel, gpsInfoLastFix) != 0 || fixColor != gpsInfoLastFixColor) {
    strncpy(gpsInfoLastFix, fixLabel, sizeof(gpsInfoLastFix) - 1);
    gpsInfoLastFix[sizeof(gpsInfoLastFix) - 1] = '\0';
    gpsInfoLastFixColor = fixColor;

    lcd.fillRect(valueRightX - 120, 82, 120, 26, bgColor);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(MR_DATUM);
    if (displayTheme == THEME_COLOR) lcd.setTextColor(fixColor, bgColor);
    else lcd.setTextColor(TFT_BLACK, bgColor);
    lcd.drawString(fixLabel, valueRightX, 95);
  }

  char qBuf[24];
  switch (fixQuality) {
    case 0:  strcpy(qBuf, "None");        break;
    case 1:  strcpy(qBuf, "GPS SPS");     break;
    case 2:  strcpy(qBuf, "DGPS");        break;
    case 4:  strcpy(qBuf, "RTK Fixed");   break;
    case 5:  strcpy(qBuf, "RTK Float");   break;
    default: snprintf(qBuf, sizeof(qBuf), "Type %d", fixQuality); break;
  }
  if (strcmp(qBuf, gpsInfoLastQuality) != 0) {
    strncpy(gpsInfoLastQuality, qBuf, sizeof(gpsInfoLastQuality) - 1);
    gpsInfoLastQuality[sizeof(gpsInfoLastQuality) - 1] = '\0';

    lcd.fillRect(valueRightX - 120, 112, 120, 26, bgColor);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(MR_DATUM);
    if (displayTheme == THEME_COLOR) lcd.setTextColor(TFT_CYAN, bgColor);
    else lcd.setTextColor(TFT_BLACK, bgColor);
    lcd.drawString(qBuf, valueRightX, 125);
  }

  if (sats != gpsInfoLastSats || valid != gpsInfoLastSatsValid) {
    gpsInfoLastSats      = sats;
    gpsInfoLastSatsValid = valid;

    char sBuf[16];
    snprintf(sBuf, sizeof(sBuf), "%d", valid ? sats : 0);

    lcd.fillRect(valueRightX - 80, 142, 80, 26, bgColor);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(MR_DATUM);
    uint16_t sc = valid ? TFT_GREEN : TFT_RED;
    if (displayTheme == THEME_COLOR) lcd.setTextColor(sc, bgColor);
    else lcd.setTextColor(TFT_BLACK, bgColor);
    lcd.drawString(sBuf, valueRightX, 155);
  }

  if (!gpsInfoConstInit ||
      hasGps != gpsInfoLastHasGps || hasGlonass != gpsInfoLastHasGlonass ||
      hasGalileo != gpsInfoLastHasGalileo || hasBds != gpsInfoLastHasBds ||
      hasQzss != gpsInfoLastHasQzss) {
    gpsInfoLastHasGps     = hasGps;
    gpsInfoLastHasGlonass = hasGlonass;
    gpsInfoLastHasGalileo = hasGalileo;
    gpsInfoLastHasBds     = hasBds;
    gpsInfoLastHasQzss    = hasQzss;
    gpsInfoConstInit      = true;

    lcd.fillRect(0, 172, 360, 106, bgColor);

    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    lcd.setTextSize((1.05f) * THEME_TEXT_SCALE, (1.05f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(MC_DATUM);

    if (displayTheme == THEME_COLOR) lcd.setTextColor(TFT_WHITE, bgColor);
    else lcd.setTextColor(TFT_BLACK, bgColor);
    lcd.drawString("Constellations:", 180, 185);

    uint16_t onColor  = displayTheme == THEME_COLOR ? TFT_GREEN :
                        (displayTheme == THEME_WHITE_BLACK ? themeVivid(TFT_GREEN) : TFT_BLUE);
    uint16_t offColor = displayTheme == THEME_COLOR ? TFT_DARKGREY : TFT_BLACK;

    lcd.setTextColor(hasGps     ? onColor : offColor, bgColor); lcd.drawString("GPS",     90, 210);
    lcd.setTextColor(hasGlonass ? onColor : offColor, bgColor); lcd.drawString("GLONASS", 250, 210);
    lcd.setTextColor(hasGalileo ? onColor : offColor, bgColor); lcd.drawString("Galileo", 90, 235);
    lcd.setTextColor(hasBds     ? onColor : offColor, bgColor); lcd.drawString("BDS",     250, 235);
    lcd.setTextColor(hasQzss    ? onColor : offColor, bgColor); lcd.drawString("QZSS",    90, 260);
  }

  if (utcH != gpsInfoLastUtcH || utcM != gpsInfoLastUtcM ||
      utcS != gpsInfoLastUtcS || valid != gpsInfoLastTimeValid) {
    gpsInfoLastUtcH      = utcH;
    gpsInfoLastUtcM      = utcM;
    gpsInfoLastUtcS      = utcS;
    gpsInfoLastTimeValid = valid;

    char timeBuf[32];
    if (valid && utcH >= 0 && utcH <= 23 && day > 0) {
      snprintf(timeBuf, sizeof(timeBuf), "UTC %02d:%02d:%02d", utcH, utcM, utcS);
    } else {
      snprintf(timeBuf, sizeof(timeBuf), "UTC --:--:--");
    }

    lcd.fillRect(0, 278, 360, 26, bgColor);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(MC_DATUM);
    if (displayTheme == THEME_COLOR) lcd.setTextColor(TFT_WHITE, bgColor);
    else lcd.setTextColor(TFT_BLACK, bgColor);
    lcd.drawString(timeBuf, 180, 292);
  }

  drawBleStatusLine();
}

// Coordinata in gradi scritta nel formato scelto ('*' = simbolo dei gradi, disegnato a mano).
void homeFormatCoord(double v, bool isLat, char *out, size_t outSize) {
  if (hcFormat == HC_FMT_GOOGLE) { snprintf(out, outSize, "%.6f", v); return; }
  const char h = isLat ? (v < 0.0 ? 'S' : 'N') : (v < 0.0 ? 'W' : 'E');
  const long unit = (hcFormat == HC_FMT_GARMIN) ? 60000L : 3600L;   // millesimi di primo / secondi
  const long r2 = (hcFormat == HC_FMT_GARMIN) ? 1000L : 60L;
  long t = (long)llround(fabs(v) * (double)unit);
  const int d = (int)(t / unit), a = (int)((t % unit) / r2), b = (int)(t % r2);
  char ds[4];
  snprintf(ds, sizeof(ds), isLat ? "%02d" : "%03d", d);
  if (hcFormat == HC_FMT_GARMIN) snprintf(out, outSize, "%c%s*%02d.%03d'", h, ds, a, b);
  else if (hcFormat == HC_FMT_XCSOAR) snprintf(out, outSize, "%c%s*%02d'%02d\"", h, ds, a, b);
  else snprintf(out, outSize, "%c%s%02d%02d", h, ds, a, b);   // SKY DEMON
}

// Riga centrata sul display con il font corrente; '*' diventa un cerchietto (i font GFX non hanno il °).
static void homeFieldDrawDegText(const char *txt, int y, uint16_t color) {
  const int degW = 10;
  char piece[32];
  int total = 0;
  for (const char *p = txt; ; ) {
    const char *star = strchr(p, '*');
    const size_t n = min(star ? (size_t)(star - p) : strlen(p), sizeof(piece) - 1);
    memcpy(piece, p, n); piece[n] = '\0';
    total += lcd.textWidth(piece);
    if (!star) break;
    total += degW; p = star + 1;
  }
  lcd.setTextDatum(ML_DATUM);
  lcd.setTextColor(color, themeBg());
  int x = 180 - total / 2;
  for (const char *p = txt; ; ) {
    const char *star = strchr(p, '*');
    const size_t n = min(star ? (size_t)(star - p) : strlen(p), sizeof(piece) - 1);
    memcpy(piece, p, n); piece[n] = '\0';
    lcd.drawString(piece, x, y);
    x += lcd.textWidth(piece);
    if (!star) break;
    lcd.drawCircle(x + degW / 2, y - 8, 3, color);
    lcd.drawCircle(x + degW / 2, y - 8, 2, color);
    x += degW; p = star + 1;
  }
  lcd.setTextDatum(MC_DATUM);
}

void drawHomeFieldScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("HOME FIELD", 180, 55, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.88f * THEME_TEXT_SCALE);

  char buf[32];
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  char label[32];
  snprintf(label, sizeof(label), "Lat (%s):", HC_FMT_NAMES[hcFormat]);   // formato scelto in COORD FORMAT
  lcd.drawString(label, 180, 75);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  setThemeTextColor(TFT_GREEN, TFT_BLACK);
  if (homePositionValid) homeFormatCoord(homeLatitude, true, buf, sizeof(buf));
  else strcpy(buf, "-- not set --");
  homeFieldDrawDegText(buf, 102, themeColor(TFT_GREEN));

  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(label, sizeof(label), "Lon (%s):", HC_FMT_NAMES[hcFormat]);
  lcd.drawString(label, 180, 142);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  setThemeTextColor(TFT_GREEN, TFT_BLACK);
  if (homePositionValid) homeFormatCoord(homeLongitude, false, buf, sizeof(buf));
  else strcpy(buf, "-- not set --");
  homeFieldDrawDegText(buf, 169, themeColor(TFT_GREEN));

  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  if (homeConfirmPending == HOME_CONFIRM_SET) {
    setThemeTextColor(TFT_ORANGE, TFT_BLACK);
    lcd.drawString("Tap SET HOME again to confirm", 180, 190);
  } else {
    setThemeTextColor(TFT_CYAN, TFT_BLACK);
    lcd.drawString("Tap center to exit", 180, 335);
  }

  const int btnY = 200, btnW = 100, btnH = 38;             // a meta' tra longitudine e riquadro GO TO
  const int setX = 68, resetX = 192;                       // centrati sulla riga, distanza 24 px
  const int goToW = 160, goToH = 61, goToX = 100, goToY = 257;
  lcd.drawRect(goToX, goToY, goToW, goToH, themeColor(TFT_GREEN));
  lcd.drawRect(setX, btnY, btnW, btnH, themeColor(TFT_ORANGE));
  lcd.drawRect(resetX, btnY, btnW, btnH, themeColor(TFT_YELLOW));
  // GO TO: stesso font VLW e stessa scala del titolo "HOME FIELD"
  drawUiVlw("GO TO", goToX + goToW / 2, goToY + goToH / 2, MC_DATUM, themeColor(TFT_GREEN), themeBg(), 0.88f * THEME_TEXT_SCALE);
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_ORANGE, TFT_BLACK);
  lcd.drawString(homeConfirmPending == HOME_CONFIRM_SET ? "CONFIRM" : "SET HOME", setX + btnW / 2, btnY + btnH / 2);
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString("COORDS", resetX + btnW / 2, btnY + btnH / 2);   // apre l'inserimento manuale delle coordinate
}

static float airfieldRwyDeg(int idx) {
  if (idx < 0 || idx >= gAirfieldCount) return 0.0f;
  const int t = gAirfields[idx].rwy;
  if (t <= 0) return 0.0f;
  float h = t * 10.0f;
  if (gAirfieldHdgOfs != nullptr && gAirfields != AIRFIELDS_FIRMWARE && gAirfieldHdgOfs[idx] != 0) h += (int)gAirfieldHdgOfs[idx] - 20;
  if (h < 0.0f) h += 360.0f;
  if (h >= 360.0f) h -= 360.0f;
  return h;
}
static int frHomeFieldIdx();   // FIELDS: campo di casa nell'elenco (definita piu' sotto)
// Pista della meta (decine di gradi, 0 = sconosciuta): il campo del GO TO oppure il campo di casa.
static int goHomeRunwayTens() {
  const int fi = navGotoActive ? navGotoIdx : frHomeFieldIdx();
  if (fi < 0 || fi >= AIRFIELD_COUNT) return 0;
  return AIRFIELDS[fi].rwy;
}
// Direzione della pista della meta in gradi (vera se il file la da'), 0 = sconosciuta
static float goHomeRunwayDeg() {
  const int fi = navGotoActive ? navGotoIdx : frHomeFieldIdx();
  if (fi < 0 || fi >= AIRFIELD_COUNT || AIRFIELDS[fi].rwy <= 0) return 0.0f;
  return airfieldRwyDeg(fi);
}

void drawGoToHomeScreen() {
  goToHomeNeedsRedraw = true;
  updateGoToHomeScreen();
}

// 0 = normale, 1 = FIELDS sta disegnando GO TO HOME, 2 = GO TO HOME sta disegnando FIELDS
static int gAltBorrow = 0;

// A terra e senza fix, si usa la posizione arrivata dal telefono invece di lasciare la pagina
// vuota. Torna true se i dati mostrati sono del telefono, cosi' la pagina lo scrive a schermo.
// Appena il GPS aggancia, gpsOk e' gia' true e questa funzione non fa niente.
// Fascia sensibile per ONLINE WEATHER: anello esterno di 30 px del display circolare.
bool windRingTapped(int x, int y) {
  const int dx = x - 180, dy = y - 180;
  const int r2 = dx * dx + dy * dy;
  return r2 >= (150 * 150) && r2 <= (180 * 180);
}

bool phonePosFill(bool &gpsOk, double &lat, double &lon) {
  if (gpsOk) return false;                     // c'e' il fix vero: non si tocca niente
  if (!metPhonePosValid || !metFresh()) return false;
  if (automaticFlightRunning) return false;    // solo a terra, a volo non iniziato
  lat = (double)metPhoneLat;
  lon = (double)metPhoneLon;
  gpsOk = true;
  return true;
}

// Il dato del telefono vale solo se e' arrivato da poco: dopo mezz'ora si smette di mostrarlo,
// perche' un vento di mezz'ora fa e' un numero sbagliato che sembra giusto.
bool metFresh() {
  if (!metWindValid && !metQnhValid) return false;
  if (metAtMs == 0) return false;
  return (millis() - metAtMs + metExtraAgeMs) < MET_FRESH_MS;
}

int metAgeMin() {
  if (metAtMs == 0) return -1;
  return (int)((millis() - metAtMs + metExtraAgeMs) / 60000UL);
}

// Il meteo che abbiamo e' di QUESTO campo? Lo e' se e' arrivato dopo l'ultimo cambio di
// destinazione. Finche' non lo e', il vento non si disegna da nessuna parte e la pagina
// ONLINE WEATHER mostra trattini: meglio niente che il numero di un altro posto.
bool metAligned() {
  // Sullo slave il conto non si puo' fare: i dati arrivano dal master una volta al secondo, e
  // guardando solo l'ora di arrivo sembrerebbero sempre nuovi. Lo dice il master nel pacchetto.
  if (bleRole == BLE_ROLE_SLAVE) return metAlignedRemote;
  if (metDestChangedMs == 0) return true;     // la destinazione non e' mai cambiata
  if (metAtMs == 0) return false;
  return (int32_t)(metAtMs - metDestChangedMs) >= 0;
}

// Per qualche secondo dopo che il dato si e' riallineato, il pallino del vento pulsa: e' il
// modo di dire "guarda qui, il vento del campo nuovo e' arrivato" senza cambiare pagina.
bool metPulseOn() {
  if (metAlignedAtMs == 0) return false;
  return (millis() - metAlignedAtMs) < MET_PULSE_MS;
}

// Vento da prendere sul serio: forte di suo, oppure con raffiche molto sopra il vento medio.
// E' la raffica a far male in finale, quindi basta una delle due per accendere il rosso.
#define MET_WIND_FORTE_KT 15.0f
#define MET_RAFFICA_KT 8.0f
bool metWindStrong() {
  if (!metWindValid) return false;
  return (metWindKt >= MET_WIND_FORTE_KT) || ((metGustKt - metWindKt) >= MET_RAFFICA_KT);
}

// Lampeggio da un secondo: la pagina FIELDS si ridisegna una volta al secondo, quindi
// un periodo piu' corto non si vedrebbe comunque.
bool metBlinkOn() { return ((millis() / 1000UL) % 2UL) == 0UL; }

// Freccia del vento sul bordo del quadrante, nelle pagine con la rotta in alto (GO TO HOME e
// FIELDS). Parte da dove soffia e punta verso il centro: se arriva da sinistra, hai vento da
// sinistra, senza doverci pensare.
void drawWindArrow(LGFX_Sprite &spr, float course, uint16_t col, int rOut, int rIn) {
  if (!metWindValid || !metFresh() || !metAligned()) return;
  const float rel = ((float)metWindDir - course) * DEG_TO_RAD;
  const float s = sinf(rel), c = cosf(rel);
  const float x0 = 180.0f + rOut * s, y0 = 180.0f - rOut * c;
  const float x1 = 180.0f + rIn * s, y1 = 180.0f - rIn * c;
  const float dx = x1 - x0, dy = y1 - y0;
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 1.0f) return;
  const float ux = dx / len, uy = dy / len;
  const float px = -uy, py = ux;
  drawThickLineSprite(spr, (int)lroundf(x0), (int)lroundf(y0), (int)lroundf(x1), (int)lroundf(y1), col, 7);
  spr.fillTriangle((int)lroundf(x1 + ux * 13.0f), (int)lroundf(y1 + uy * 13.0f),
                   (int)lroundf(x1 + px * 8.0f),  (int)lroundf(y1 + py * 8.0f),
                   (int)lroundf(x1 - px * 8.0f),  (int)lroundf(y1 - py * 8.0f), col);
  spr.fillCircle((int)lroundf(x0), (int)lroundf(y0), 4, col);
}

void updateGoToHomeScreen() {
  if (currentScreen != GO_TO_HOME_SCREEN) return;
  windDotShown = false;   // riempito piu' sotto solo se il pallino del vento viene disegnato

  bool gpsValid = false;
  double lat = 0.0, lon = 0.0;
  float course = 0.0f, speedKmh = 0.0f;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    gpsValid = gpsDataValid;
    lat = gps_latitude;
    lon = gps_longitude;
    course = gps_course;
    speedKmh = gps_ground_speed_kmh;
    xSemaphoreGive(sensorDataMutex);
  } else return;

  const int gpsState = gpsValid ? 1 : 0;
  double distanceKm = 0.0;
  double bearing = 0.0;
  int bearingDeg = -999;
  int courseDeg = (int)round(course);
  courseDeg %= 360;
  if (courseDeg < 0) courseDeg += 360;
  int distanceTenth = -999;
  long etaSeconds = -1;
  const bool navigationValid = navTargetValid() && gpsValid;   // casa oppure campo scelto (GO TO)
  float xteKm = 0.0f;
  bool xteValid = false;
  int xteState = 0;   // 0 n/d, 1 dentro, 2 dx giallo, 3 dx rosso, 4 sx giallo, 5 sx rosso
  int arrowStage = 6;

  if (navigationValid) {
    const double lat1 = lat * DEG_TO_RAD;
    const double tLat = navTargetLat(), tLon = navTargetLon();
    const double lat2 = tLat * DEG_TO_RAD;
    const double dLat = (tLat - lat) * DEG_TO_RAD;
    const double dLon = (tLon - lon) * DEG_TO_RAD;
    const double a = sin(dLat / 2.0) * sin(dLat / 2.0) + cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);
    distanceKm = 6371.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    bearing = atan2(sin(dLon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon)) / DEG_TO_RAD;
    if (bearing < 0.0) bearing += 360.0;
    bearingDeg = (int)round(bearing);
    bearingDeg %= 360;
    distanceTenth = (int)round(distanceKm * 10.0);
    const double courseToHome = (course - bearing) * DEG_TO_RAD;
    const double closingSpeedKmh = speedKmh * cos(courseToHome);
    etaSeconds = closingSpeedKmh > 1.0 ? (long)ceil(distanceKm / closingSpeedKmh * 3600.0) : -1;
    if (!lubberLegValid) { lubberLegLat = lat; lubberLegLon = lon; lubberLegValid = true; }
    // Arrivo a casa: oltre 2 km freccia fissa; da 2 km a 1.5 km la freccia lampeggia; poi il bersaglio HOME:
    // rosso (1.5 km - 900 m), si aggiunge il giallo (900-500 m), si aggiunge il verde lampeggiante (500-400 m),
    // poi verde fisso non lampeggiante (400-0 m). I cerchi si sommano.
    const bool blinkPhase = ((millis() / 300UL) % 2UL == 0UL);
    if (distanceKm > 2.0) arrowStage = 6;                          // freccia fissa
    else if (distanceKm >= 1.5) arrowStage = blinkPhase ? 7 : 8;   // freccia lampeggiante
    else if (distanceKm >= 0.9) arrowStage = 9;                    // rosso (1.5 km - 900 m)
    else if (distanceKm >= 0.5) arrowStage = 10;                   // + giallo (900-500 m)
    else if (distanceKm >= 0.4) arrowStage = blinkPhase ? 11 : 12; // + verde lampeggiante (500-400 m)
    else arrowStage = 13;                                          // + verde fisso (400-0 m)
    lubberReanchorIfNeeded(lat, lon);
    xteValid = lubberCrossTrackKm(lat, lon, xteKm);
    double courseDiff = fmod((double)course - bearing, 360.0);
    if (courseDiff < 0.0) courseDiff += 360.0;
    if (courseDiff > 180.0) courseDiff = 360.0 - courseDiff;
    if (speedKmh >= LUBBER_MIN_SPEED_KMH) {
      if (lubberActive && courseDiff > LUBBER_OFF_DEG) lubberActive = false;
      else if (!lubberActive && courseDiff < LUBBER_ON_DEG) lubberActive = true;
    }
    if (xteValid && lubberActive) {
      // Imbuto automatico: le impostazioni valgono a LUBBER_FUNNEL_REF_KM o piu' da casa; sotto,
      // verde e giallo si riducono in proporzione alla distanza residua (con un minimo).
      // LUBBER LINE spenta in PAGES: valori fissi verde 1 km, giallo +1 km
      float widthKm = LUBBER_WIDTH_OPTIONS_KM[lubberEditEnabled ? lubberWidthIdx : 0];
      float yellowKm = LUBBER_YELLOW_OPTIONS_KM[lubberEditEnabled ? lubberYellowIdx : 0];
      if (distanceKm < LUBBER_FUNNEL_REF_KM) {
        const float k = (float)(distanceKm / LUBBER_FUNNEL_REF_KM);
        widthKm = fminf(widthKm, fmaxf(widthKm * k, LUBBER_MIN_GREEN_KM));
        yellowKm = fminf(yellowKm, fmaxf(yellowKm * k, LUBBER_MIN_YELLOW_KM));
      }
      float absXte = fabsf(xteKm);
      // Fuori corridoio ma la rotta punta di nuovo verso casa (entro LUBBER_REANCHOR_DEG): la tratta
      // riparte dalla posizione attuale, quindi lo scostamento torna 0 e le linee tornano verdi.
      if (absXte > widthKm && courseDiff < LUBBER_REANCHOR_DEG && speedKmh >= LUBBER_MIN_SPEED_KMH &&
          distanceKm >= LUBBER_MIN_LEG_KM) {
        lubberLegLat = lat;
        lubberLegLon = lon;
        xteKm = 0.0f;
        absXte = 0.0f;
      }
      if (absXte <= widthKm) xteState = 1;
      else xteState = (xteKm > 0.0f ? 2 : 4) + (absXte > widthKm + yellowKm ? 1 : 0);
    }
  }

  // In avvicinamento (dal bersaglio rosso in poi, sotto 1.5 km) questa pagina si alterna con la vista
  // della pista di FIELDS: 5 s GO TO HOME, 3 s la pista. gAltBorrow evita il rimpallo fra le due pagine.
  if (gAltBorrow == 0 && navigationValid && arrowStage >= 9) {
    static bool goFieldsShown = false;
    if ((millis() % 8000UL) >= 5000UL) {
      goFieldsShown = true;
      gAltBorrow = 2;
      currentScreen = FIELDS_RADAR_SCREEN;   // la pagina FIELDS disegna solo se e' quella corrente
      updateFieldsRadarScreen(true);
      currentScreen = GO_TO_HOME_SCREEN;
      gAltBorrow = 0;
      return;
    }
    if (goFieldsShown) { goFieldsShown = false; goToHomeNeedsRedraw = true; }
  }

  if (!goToHomeNeedsRedraw && gpsState == lastGoToHomeGpsState && bearingDeg == lastGoToHomeBearingDeg &&
      courseDeg == lastGoToHomeCourseDeg && distanceTenth == lastGoToHomeDistanceTenth &&
      etaSeconds == lastGoToHomeEtaSeconds && xteState == lastGoToHomeXteState &&
      arrowStage == lastGoToHomeArrowStage) return;

  const unsigned long now = millis();
  if (!goToHomeNeedsRedraw && now - lastGoToHomeRedrawMillis < GO_TO_HOME_REDRAW_INTERVAL_MS) return;

  const uint16_t bgColor = themeBg();
  const uint16_t whiteColor = themeColor(TFT_WHITE);
  const uint16_t cyanColor = themeColor(TFT_CYAN);
  const uint16_t yellowColor = themeColor(TFT_YELLOW);
  const uint16_t lubberColor = themeColor(TFT_WHITE);
  if (!ensureNavigationSprite()) {
    lcd.fillScreen(bgColor);
    lcd.setTextDatum(MC_DATUM);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize(1, 1);
    lcd.setTextColor(themeColor(TFT_RED), bgColor);
    lcd.drawString("MEMORIA INSUFFICIENTE", 180, 160);
    lcd.drawString("NAVIGAZIONE NON DISPONIBILE", 180, 200);
    return;
  }
  navigationSprite.fillSprite(bgColor);
  // Linee lubber = corridoio fuori rotta: verdi dentro la larghezza impostata; la linea del lato da
  // cui si esce diventa gialla (fino al margine) poi rossa, e piu' spessa. Senza dati restano bianche.
  uint16_t leftLineColor = lubberColor, rightLineColor = lubberColor;
  int leftLineThick = 2, rightLineThick = 2;
  int correctDir = 0;               // +1 correggere verso destra, -1 verso sinistra
  uint16_t correctColor = lubberColor;
  if (xteState >= 1) {
    leftLineColor = rightLineColor = themeVivid(TFT_GREEN);
    if (xteState >= 2) {
      const bool isRed = (xteState == 3 || xteState == 5);
      const uint16_t warnColor = themeVivid(isRed ? TFT_RED : TFT_YELLOW);
      const int warnThick = isRed ? 6 : 4;
      if (xteState <= 3) { rightLineColor = warnColor; rightLineThick = warnThick; }
      else { leftLineColor = warnColor; leftLineThick = warnThick; }
      correctColor = warnColor;
    }
  }
  // Freccetta: da che parte virare per portare la freccia grande al centro tra le linee lubber
  // (verso il campo). Solo con la linea gialla o rossa (fuori corridoio), dello stesso colore, oltre 10 gradi.
  if (bearingDeg != -999 && xteState >= 2) {
    int rel = ((bearingDeg - courseDeg) % 360 + 540) % 360 - 180;
    if (rel > 10) correctDir = 1;
    else if (rel < -10) correctDir = -1;
  }
  // Dal primo cerchio rosso (sotto 1.5 km) le linee lubber diventano la pista della meta:
  // ruotano con la rotta (rotta in alto) e alle testate compaiono i numeri della pista.
  const int rwyTens = (navigationValid && arrowStage >= 9) ? goHomeRunwayTens() : 0;
  if (rwyTens > 0) {
    const float ra = (goHomeRunwayDeg() - course) * DEG_TO_RAD;   // 0 = pista verticale sul display
    const float ux = sinf(ra), uy = -cosf(ra);                  // lungo la pista (verso rwy)
    const float px = -uy, py = ux;                               // di lato (a destra se ux, uy punta in alto)
    const float half = 172.0f;
    const float offL = -(8.0f + leftLineThick / 2.0f), offR = 10.0f + rightLineThick / 2.0f;
    navigationSprite.drawWideLine(180.0f + px * offL - ux * half, 180.0f + py * offL - uy * half,
                                  180.0f + px * offL + ux * half, 180.0f + py * offL + uy * half,
                                  leftLineThick / 2.0f, leftLineColor);
    navigationSprite.drawWideLine(180.0f + px * offR - ux * half, 180.0f + py * offR - uy * half,
                                  180.0f + px * offR + ux * half, 180.0f + py * offR + uy * half,
                                  rightLineThick / 2.0f, rightLineColor);
  } else {
    navigationSprite.fillRect(172 - leftLineThick, 8, leftLineThick, 345, leftLineColor);
    navigationSprite.fillRect(190, 8, rightLineThick, 345, rightLineColor);
  }

  if (!navigationValid) {
    navigationSprite.setTextDatum(MC_DATUM);
    // Titolo nel quarto alto del display e abbastanza stretto da non uscire dal bordo tondo.
    navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    navigationSprite.setTextSize((1.2f) * THEME_TEXT_SCALE, (1.2f) * THEME_TEXT_SCALE);
    navigationSprite.setTextColor(displayTheme == THEME_COLOR ? TFT_CYAN : TFT_BLACK, bgColor);
    if (navGotoActive) {   // GO TO verso un campo: nome del campo in arancione
      char t[32];
      snprintf(t, sizeof(t), "GO TO %s", AIRFIELDS[navGotoIdx].name);
      navigationSprite.setTextSize((0.85f) * THEME_TEXT_SCALE, (0.85f) * THEME_TEXT_SCALE);
      navigationSprite.setTextColor(themeColor(TFT_ORANGE), bgColor);
      navigationSprite.drawString(t, 180, 62);
    } else {
      navigationSprite.drawString("GO TO HOME", 180, 62);
    }
    if (navTargetValid()) {
      // Senza fix, ma col telefono collegato e a terra, almeno la DISTANZA si puo' dare: dipende
      // solo dalla posizione. La DIREZIONE no: questa pagina ha la rotta in alto, e da fermi la
      // rotta non esiste, quindi la freccia resta spenta e si scrive solo quanto manca.
      bool pOk = false; double pLat = 0.0, pLon = 0.0;
      const bool daTelefono = phonePosFill(pOk, pLat, pLon);
      if (daTelefono) {
        const double la1 = pLat * DEG_TO_RAD, lo1 = pLon * DEG_TO_RAD;
        const double la2 = navTargetLat() * DEG_TO_RAD, lo2 = navTargetLon() * DEG_TO_RAD;
        const double dla = la2 - la1, dlo = lo2 - lo1;
        const double a = sin(dla / 2) * sin(dla / 2) +
                         cos(la1) * cos(la2) * sin(dlo / 2) * sin(dlo / 2);
        const double km = 6371.0 * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
        char db[20];
        if (km < 100.0) snprintf(db, sizeof(db), "%.1f km", km);
        else snprintf(db, sizeof(db), "%.0f km", km);
        drawUiVlwSprite(navigationSprite, db, 180, 196, MC_DATUM,
                        themeColor(TFT_WHITE), bgColor, 1.00f);
        drawUiVlwSprite(navigationSprite, "POSITION FROM PHONE", 180, 238, MC_DATUM,
                        themeColor(TFT_DARKGREY), bgColor, 0.36f);
        drawUiVlwSprite(navigationSprite, "waiting for GPS", 180, 262, MC_DATUM,
                        themeColor(TFT_DARKGREY), bgColor, 0.36f);
      } else {
        // Manca il GPS: stessa scritta, carattere, grandezza, colore e posizione della pagina bussola.
        navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
        navigationSprite.setTextSize((1.35f) * THEME_TEXT_SCALE, (1.35f) * THEME_TEXT_SCALE);
        navigationSprite.setTextColor(themeColor(TFT_RED), bgColor);
        navigationSprite.drawString("NO GPS FIX", 180, 215);
      }
    } else {
      // Mancano le coordinate di casa: "Tap to center to set home" su 2 righe,
      // stessa grandezza e colore di "HOME NOT SET".
      navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
      navigationSprite.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
      navigationSprite.setTextColor(displayTheme == THEME_COLOR ? TFT_YELLOW : TFT_BLACK, bgColor);
      navigationSprite.drawString("HOME NOT SET", 180, 150);
      navigationSprite.drawString("Tap to center", 180, 198);
      navigationSprite.drawString("to set home", 180, 230);
    }

  } else {
    const int cx = 180, cy = 180;
    const int tickRadius = 175;
    const int numRadius = 145;
    navigationSprite.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    for (int angle = 0; angle < 360; angle += 10) {
      const float rad = (angle - course - 90.0f) * DEG_TO_RAD;
      const int xOuter = cx + tickRadius * cos(rad);
      const int yOuter = cy + tickRadius * sin(rad);
      if (angle % 30 == 0) {
        const int xInner = cx + (tickRadius - 20) * cos(rad);
        const int yInner = cy + (tickRadius - 20) * sin(rad);
        drawTickLine15Sprite(navigationSprite, xInner, yInner, xOuter, yOuter, whiteColor, bgColor);
        const int numX = cx + numRadius * cos(rad);
        const int numY = cy + numRadius * sin(rad);
        navigationSprite.setTextDatum(MC_DATUM);
        navigationSprite.setTextColor(whiteColor, bgColor);
        if (angle == 0 || angle == 90 || angle == 180 || angle == 270) {
          navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
          navigationSprite.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
          const char* cardinal = angle == 0 ? "N" : (angle == 90 ? "E" : (angle == 180 ? "S" : "W"));
          navigationSprite.setTextColor(angle == 0 ? themeColor(TFT_ORANGE) : whiteColor, bgColor);
          navigationSprite.drawString(cardinal, numX, numY);
        } else {
          char degreeBuf[4];
          snprintf(degreeBuf, sizeof(degreeBuf), "%d", angle / 10);
          drawNumericVlwSprite(navigationSprite, degreeBuf, numX, numY, MC_DATUM, TFT_WHITE, bgColor, 0.34f);
        }
      } else {
        const int xInner = cx + (tickRadius - 10) * cos(rad);
        const int yInner = cy + (tickRadius - 10) * sin(rad);
        drawTickLine15Sprite(navigationSprite, xInner, yInner, xOuter, yOuter, whiteColor, bgColor);
      }
    }

    const float arrowAngle = (bearing - course - 90.0) * DEG_TO_RAD;
    const int tipX = cx + 124 * cos(arrowAngle);
    const int tipY = cy + 124 * sin(arrowAngle);
    const int neckX = cx + 84 * cos(arrowAngle);
    const int neckY = cy + 84 * sin(arrowAngle);
    const int baseX = cx + 14 * cos(arrowAngle);
    const int baseY = cy + 14 * sin(arrowAngle);
    const float leftA = arrowAngle + 2.55f;
    const float rightA = arrowAngle - 2.55f;

    const uint16_t compassCyan = themeColor(TFT_CYAN);
    // Triangolo superiore: come in COMPASS_SCREEN ma capovolto, apice rivolto verso il basso
    drawThickTriangleSprite(navigationSprite, cx, 27, cx - 8, 7, cx + 8, 7,
                            compassCyan, 3);
    // Triangolo inferiore: identico a COMPASS_SCREEN, apice rivolto verso il basso
    drawThickTriangleSprite(navigationSprite, cx, 353, cx - 8, 333, cx + 8, 333,
                            compassCyan, 3);
    char buf[32];
    const int navLeftX = cx - 60;   // Km to: 30px piu a dx rispetto a prima
    const int navRightX = cx + 60;  // ETA: 30px piu a sx rispetto a prima
    const int navLabelY = cy - 20;
    const int navValueY = cy + 22;

    // Dal primo cerchio rosso (sotto 1.5 km) Km to ed ETA spariscono per lasciare spazio al bersaglio.
    if (arrowStage < 9) {
      // "Km to" a sinistra, sotto il solo numero dei km mancanti (senza unita' "km")
      drawUiVlwSprite(navigationSprite, "Km to", navLeftX, navLabelY, MC_DATUM, TFT_YELLOW, bgColor, 0.87f);
      snprintf(buf, sizeof(buf), "%.1f", distanceKm);
      drawUiVlwSprite(navigationSprite, buf, navLeftX, navValueY, MC_DATUM, TFT_YELLOW, bgColor, 1.05f);

      // "ETA" a destra, sotto il tempo in ore:minuti
      drawUiVlwSprite(navigationSprite, "ETA", navRightX, navLabelY, MC_DATUM, TFT_YELLOW, bgColor, 0.87f);
      if (etaSeconds >= 0) {
        const long etaHours = etaSeconds / 3600;
        const int etaRemainderMinutes = (int)((etaSeconds % 3600) / 60);
        snprintf(buf, sizeof(buf), "%02ld:%02d", etaHours, etaRemainderMinutes);
      } else strcpy(buf, "--:--");
      drawUiVlwSprite(navigationSprite, buf, navRightX, navValueY, MC_DATUM, TFT_YELLOW, bgColor, 1.05f);
      drawUiVlwSprite(navigationSprite, "HH", navRightX - 21, navValueY + 21,
                      MC_DATUM, TFT_YELLOW, bgColor, 0.48f);
      drawUiVlwSprite(navigationSprite, "MM", navRightX + 21, navValueY + 21,
                      MC_DATUM, TFT_YELLOW, bgColor, 0.48f);
    }

    // Asta della freccia prolungata anche dal lato opposto alla punta, quasi a toccare i numeri della bussola (numRadius = 145)
    const int tailBaseX = cx - 14 * cos(arrowAngle);
    const int tailBaseY = cy - 14 * sin(arrowAngle);
    const int tailEndX = cx - 140 * cos(arrowAngle);
    const int tailEndY = cy - 140 * sin(arrowAngle);
    auto drawTip = [&]() {
      navigationSprite.fillTriangle(tipX, tipY, neckX + 25 * cos(leftA), neckY + 25 * sin(leftA),
                                    neckX + 25 * cos(rightA), neckY + 25 * sin(rightA), yellowColor);
    };
    auto drawFullArrow = [&]() {
      drawThickLineSprite(navigationSprite, tailBaseX, tailBaseY, tailEndX, tailEndY, yellowColor, 11);
      drawThickLineSprite(navigationSprite, baseX, baseY, neckX, neckY, yellowColor, 11);
      drawTip();
    };
    if (arrowStage < 9) {
      if (arrowStage != 8) drawFullArrow();   // 8 = fase spenta del lampeggio
      navigationSprite.fillCircle(cx, cy, 8, cyanColor);
    } else {
      // Bersaglio HOME al centro del display (diametro doppio). In alto la rotta opposta, in basso la rotta,
      // entrambe in verde e in decine di gradi su 2 cifre come nella bussola (90 = 09, 180 = 18, nord = 36).
      // I cerchi non ancora accesi restano come contorno sottile grigio.
      const int tx = cx, ty = cy;
      const uint16_t ghostColor = themeColor(TFT_DARKGREY);
      const uint16_t redRing = themeColor(TFT_RED);
      const uint16_t greenRing = themeVivid(TFT_GREEN);
      const bool yellowOn = arrowStage >= 10;
      const bool greenOn = (arrowStage == 11 || arrowStage == 13);   // 12 = fase spenta del lampeggio verde, 13 = verde fisso
      navigationSprite.fillCircle(tx, ty, 76, redRing);          // rosso: acceso da 1.5 km
      navigationSprite.fillCircle(tx, ty, 62, bgColor);
      if (yellowOn) {
        navigationSprite.fillCircle(tx, ty, 50, yellowColor);    // giallo: aggiunto da 900 m
        navigationSprite.fillCircle(tx, ty, 36, bgColor);
      } else {
        navigationSprite.drawCircle(tx, ty, 43, ghostColor);
      }
      if (greenOn) navigationSprite.fillCircle(tx, ty, 24, greenRing);   // verde pieno: lampeggiante 500-400 m, fisso sotto 400 m
      else navigationSprite.drawCircle(tx, ty, 24, ghostColor);
      if (rwyTens > 0) {
        // Pista sopra i cerchi rosso, giallo e verde: le linee non spariscono (fino ai numeri della bussola).
        const float ra = (rwyTens * 10.0f - course) * DEG_TO_RAD;
        const float ux = sinf(ra), uy = -cosf(ra);
        const float px = -uy, py = ux;
        const float half = 132.0f;
        const float offL = -(8.0f + leftLineThick / 2.0f), offR = 10.0f + rightLineThick / 2.0f;
        navigationSprite.drawWideLine(180.0f + px * offL - ux * half, 180.0f + py * offL - uy * half,
                                      180.0f + px * offL + ux * half, 180.0f + py * offL + uy * half,
                                      leftLineThick / 2.0f, leftLineColor);
        navigationSprite.drawWideLine(180.0f + px * offR - ux * half, 180.0f + py * offR - uy * half,
                                      180.0f + px * offR + ux * half, 180.0f + py * offR + uy * half,
                                      rightLineThick / 2.0f, rightLineColor);
      } else {
        int courseTens = (int)((course + 5.0f) / 10.0f) % 36;
        if (courseTens == 0) courseTens = 36;
        int oppositeTens = (courseTens + 18) % 36;
        if (oppositeTens == 0) oppositeTens = 36;
        char courseTensBuf[4], oppositeTensBuf[4];
        snprintf(courseTensBuf, sizeof(courseTensBuf), "%02d", courseTens);
        snprintf(oppositeTensBuf, sizeof(oppositeTensBuf), "%02d", oppositeTens);
        drawUiVlwSprite(navigationSprite, oppositeTensBuf, cx, cy - 114, MC_DATUM, greenRing, bgColor, 0.95f);
        drawUiVlwSprite(navigationSprite, courseTensBuf, cx, cy + 114, MC_DATUM, greenRing, bgColor, 0.95f);
      }
      // Freccia verso casa ridotta al diametro del cerchio rosso (r = 76), sopra i cerchi:
      // resta con il rosso e con il giallo, sparisce quando compare il cerchio verde.
      if (arrowStage == 9 || arrowStage == 10) {
        const float ca = cosf(arrowAngle), sa = sinf(arrowAngle);
        const int sTipX = cx + (int)lroundf(74 * ca), sTipY = cy + (int)lroundf(74 * sa);
        const int sNeckX = cx + (int)lroundf(48 * ca), sNeckY = cy + (int)lroundf(48 * sa);
        const int sTailX = cx - (int)lroundf(70 * ca), sTailY = cy - (int)lroundf(70 * sa);
        const float lA = arrowAngle + 2.55f, rA = arrowAngle - 2.55f;
        const int hx1 = sNeckX + (int)lroundf(16 * cosf(lA)), hy1 = sNeckY + (int)lroundf(16 * sinf(lA));
        const int hx2 = sNeckX + (int)lroundf(16 * cosf(rA)), hy2 = sNeckY + (int)lroundf(16 * sinf(rA));
        // bordo scuro, cosi' la freccia bianca si stacca bene dai cerchi
        drawThickLineSprite(navigationSprite, sTailX, sTailY, sNeckX, sNeckY, bgColor, 11);
        const int ox1 = sNeckX + (int)lroundf(20 * cosf(lA)), oy1 = sNeckY + (int)lroundf(20 * sinf(lA));
        const int ox2 = sNeckX + (int)lroundf(20 * cosf(rA)), oy2 = sNeckY + (int)lroundf(20 * sinf(rA));
        navigationSprite.fillTriangle(cx + (int)lroundf(80 * ca), cy + (int)lroundf(80 * sa), ox1, oy1, ox2, oy2, bgColor);
        drawThickLineSprite(navigationSprite, sTailX, sTailY, sNeckX, sNeckY, whiteColor, 7);
        navigationSprite.fillTriangle(sTipX, sTipY, hx1, hy1, hx2, hy2, whiteColor);
      }
      if (rwyTens > 0) {
        // Numeri della pista alle testate, in primo piano sopra la pista: 30 % piu' grandi, in una cornice verde,
        // centrati tra i numeri della bussola (r 145) e il cerchio rosso (r 76).
        // La testata "rwy" e' quella da cui si atterra con prua rwy: sta dal lato opposto alla sua direzione
        // (in basso quando si e' allineati in finale).
        const int r1 = rwyTens, r2 = (rwyTens + 18 - 1) % 36 + 1;
        const float ra = (rwyTens * 10.0f - course) * DEG_TO_RAD;
        const float ux = sinf(ra), uy = -cosf(ra);
        const float NUM_SCALE = 0.95f * 1.3f, NUM_R = 105.0f;
        for (int e = 0; e < 2; ++e) {
          char b[4];
          snprintf(b, sizeof(b), "%02d", e == 0 ? r1 : r2);
          const float sgn = e == 0 ? -1.0f : 1.0f;
          const int nx = cx + (int)lroundf(sgn * NUM_R * ux), ny = cy + (int)lroundf(sgn * NUM_R * uy);
          const int w = uiVlwWidthSprite(navigationSprite, b, NUM_SCALE) + 14;
          const int h = (int)lroundf(26.0f * NUM_SCALE) + 12;   // 4 px piu' alta: il numero non tocca i bordi
          navigationSprite.fillRoundRect(nx - w / 2, ny - h / 2, w, h, 7, bgColor);
          drawUiVlwSprite(navigationSprite, b, nx, ny + 1, MC_DATUM, greenRing, bgColor, NUM_SCALE);
          // cornice dopo il numero: lo sfondo della scritta non copre piu' il bordo sotto
          navigationSprite.drawRoundRect(nx - w / 2, ny - h / 2, w, h, 7, greenRing);
          navigationSprite.drawRoundRect(nx - w / 2 + 1, ny - h / 2 + 1, w - 2, h - 2, 6, greenRing);
        }
      }
    }

    // GO TO verso un campo: nome del campo (arancione) e pulsante HOME per tornare alla navigazione verso casa.
    if (navGotoActive && arrowStage < 9) {
      const uint16_t orange = themeColor(TFT_ORANGE);
      navigationSprite.fillRoundRect(cx - 82, 233, 164, 28, 8, bgColor);
      navigationSprite.drawRoundRect(cx - 82, 233, 164, 28, 8, orange);
      drawUiVlwSprite(navigationSprite, AIRFIELDS[navGotoIdx].name, cx, 247, MC_DATUM, TFT_ORANGE, bgColor, 0.55f);
      navigationSprite.fillRoundRect(cx - 44, 286, 88, 28, 8, bgColor);
      navigationSprite.drawRoundRect(cx - 44, 286, 88, 28, 8, yellowColor);
      navigationSprite.drawRoundRect(cx - 43, 287, 86, 26, 7, yellowColor);
      {   // HOME grande, quasi a toccare il riquadro (largo 86 px dentro, alto 26)
        float hs = 0.84f;
        if (vlwSet(navigationSprite, UiFont32)) {
          navigationSprite.setTextSize(1.0f, 1.0f);
          const int w1 = navigationSprite.textWidth("HOME");
          vlwRelease(navigationSprite);
          if (w1 > 0 && hs * w1 > 78.0f) hs = 78.0f / w1;
        }
        drawUiVlwSprite(navigationSprite, "HOME", cx, 300, MC_DATUM, TFT_YELLOW, bgColor, hs);
      }
    }

    // Piccola freccia: da che parte virare (verso la freccia grande). Sta dal lato opposto delle linee:
    // a x=130 (a sinistra) se punta a destra, a x=230 (a destra) se punta a sinistra.
    if (correctDir != 0 && arrowStage < 9) {
      const int ay = 125;
      const int ax = (correctDir < 0) ? 230 : 130;
      drawThickLineSprite(navigationSprite, ax - correctDir * 23, ay, ax + correctDir * 5, ay, correctColor, 5);
      navigationSprite.fillTriangle(ax + correctDir * 23, ay, ax + correctDir * 5, ay - 13,
                                    ax + correctDir * 5, ay + 13, correctColor);
    }
  }
  // Vento dal telefono (app AVEA Link): freccia sul bordo, da dove soffia verso il centro.
  // Se il telefono non c'e' o il dato e' vecchio, qui non viene disegnato niente.
  if (metWindValid && metFresh() && metAligned() && navigationValid) {
    const uint16_t windCol = metWindStrong()
        ? ((bgColor == TFT_BLACK) ? lgfx::color565(230, 60, 60) : lgfx::color565(170, 0, 0))
        : ((bgColor == TFT_BLACK) ? lgfx::color565(61, 209, 122) : lgfx::color565(0, 105, 55));
    // Freccia un po' piu' corta per far posto al pallino con i nodi sulla coda, come nella
    // vista pista: tutto sulla corona esterna, lontano dalla freccia di casa e dalle lubber line.
    drawWindArrow(navigationSprite, course, windCol, 140, 100);
    const float relW = ((float)metWindDir - course) * DEG_TO_RAD;
    const int wcx = (int)lroundf(180.0f + 140.0f * sinf(relW));
    const int wcy = (int)lroundf(180.0f - 140.0f * cosf(relW));
    // Anelli che pulsano per qualche secondo dopo un cambio di campo: dato nuovo da guardare.
    // Sono BLU di proposito: il rosso sul pallino significa gia' vento forte o a raffiche, e
    // usarlo anche per "dato nuovo" renderebbe ambigue entrambe le cose.
    if (metPulseOn()) {
      const uint16_t pc = (bgColor == TFT_BLACK) ? lgfx::color565(61, 165, 244)
                                                 : lgfx::color565(0, 70, 160);
      navigationSprite.drawCircle(wcx, wcy, 26, pc);
      navigationSprite.drawCircle(wcx, wcy, 33, pc);
    }
    navigationSprite.fillCircle(wcx, wcy, 19, TFT_WHITE);
    navigationSprite.drawCircle(wcx, wcy, 19, windCol);
    navigationSprite.drawCircle(wcx, wcy, 18, windCol);
    windDotX = wcx; windDotY = wcy; windDotShown = true;
    if (!metWindStrong() || metBlinkOn()) {
      char wkb[8];
      snprintf(wkb, sizeof(wkb), "%d", (int)lroundf((float)metWindKt));
      drawUiVlwSprite(navigationSprite, wkb, wcx, wcy + 1, MC_DATUM, windCol, TFT_WHITE, 0.52f);
    }
  }
  navigationSprite.pushSprite(0, 0); drawPageLockOverlay();

  lastGoToHomeGpsState = gpsState;
  lastGoToHomeBearingDeg = bearingDeg;
  lastGoToHomeCourseDeg = courseDeg;
  lastGoToHomeDistanceTenth = distanceTenth;
  lastGoToHomeEtaSeconds = etaSeconds;
  lastGoToHomeXteState = xteState;
  lastGoToHomeArrowStage = arrowStage;
  lastGoToHomeRedrawMillis = now;
  goToHomeNeedsRedraw = false;
}

// ========== NEAREST FIELDS: i 3 campi di volo piu' vicini (AirfieldsAVEA.h) ==========
// Per ogni campo: freccia verso il campo rispetto alla rotta (in alto = davanti), nome, distanza,
// pista (testate e lunghezza) e frequenza. Tocco su un campo: GO TO, cioe' GO HOME guida verso quel campo.
static const int NF_ROW_Y0 = 78, NF_ROW_STEP = 72, NF_ROW_H = 64, NF_ROW_X = 34, NF_ROW_W = 292;
static const int NF_SEARCH_X = 118, NF_SEARCH_Y = 296, NF_SEARCH_W = 124, NF_SEARCH_H = 34;   // pulsante SEARCH
static int nfIdx[3] = {-1, -1, -1};     // campi mostrati (indici in AIRFIELDS)
static float nfDistKm[3] = {0, 0, 0};
static float nfBearing[3] = {0, 0, 0};
static unsigned long nfLastDrawMs = 0;
// Scorrimento: swipe in su = 3 campi successivi, swipe in giu' = indietro (fino al 30esimo campo).
static const int NF_MAX = 30;
static int nfOffset = 0;                // 0 = campi 1-3, 3 = campi 4-6, ...
static int nfFound = 0;                 // campi trovati (al massimo NF_MAX)

// Cerca i campi piu' vicini (distanza piana: basta per l'ordine, poi distanza e rotta esatte per i 3 mostrati).
static void nearestFieldsSearch(double lat, double lon) {
  const double kLon = cos(lat * DEG_TO_RAD);
  float best[NF_MAX];
  int bi[NF_MAX];
  for (int k = 0; k < NF_MAX; ++k) { best[k] = 1e9f; bi[k] = -1; }
  for (int i = 0; i < AIRFIELD_COUNT; ++i) {
    const float dy = (float)(AIRFIELDS[i].lat5 / 100000.0 - lat);
    const float dx = (float)((AIRFIELDS[i].lon5 / 100000.0 - lon) * kLon);
    const float d2 = dx * dx + dy * dy;
    if (d2 >= best[NF_MAX - 1]) continue;
    int k = NF_MAX - 1;
    while (k > 0 && d2 < best[k - 1]) { best[k] = best[k - 1]; bi[k] = bi[k - 1]; --k; }
    best[k] = d2; bi[k] = i;
  }
  nfFound = 0;
  while (nfFound < NF_MAX && bi[nfFound] >= 0) ++nfFound;
  if (nfOffset > nfFound - 1) nfOffset = max(0, ((nfFound - 1) / 3) * 3);
  for (int k = 0; k < 3; ++k) {
    nfIdx[k] = (nfOffset + k < NF_MAX) ? bi[nfOffset + k] : -1;
    if (nfIdx[k] < 0) continue;
    const double la1 = lat * DEG_TO_RAD, lo1 = lon * DEG_TO_RAD;
    const double la2 = AIRFIELDS[nfIdx[k]].lat5 / 100000.0 * DEG_TO_RAD, lo2 = AIRFIELDS[nfIdx[k]].lon5 / 100000.0 * DEG_TO_RAD;
    nfDistKm[k] = (float)(lubberAngDist(la1, lo1, la2, lo2) * 6371.0);
    float b = (float)(lubberBearing(la1, lo1, la2, lo2) / DEG_TO_RAD);
    if (b < 0.0f) b += 360.0f;
    nfBearing[k] = b;
  }
}

static void nfDrawArrow(lgfx::LGFXBase &g, int cx, int cy, float relDeg, uint16_t color, uint16_t ring) {
  g.drawCircle(cx, cy, 19, ring);
  g.drawCircle(cx, cy, 18, ring);
  const float a = (relDeg - 90.0f) * DEG_TO_RAD, r = 14.0f;
  const int tx = cx + (int)lroundf(r * cosf(a)), ty = cy + (int)lroundf(r * sinf(a));
  const int b1x = cx + (int)lroundf(r * 0.8f * cosf(a + 2.5f)), b1y = cy + (int)lroundf(r * 0.8f * sinf(a + 2.5f));
  const int b2x = cx + (int)lroundf(r * 0.8f * cosf(a - 2.5f)), b2y = cy + (int)lroundf(r * 0.8f * sinf(a - 2.5f));
  const int mx = cx - (int)lroundf(r * 0.25f * cosf(a)), my = cy - (int)lroundf(r * 0.25f * sinf(a));
  g.fillTriangle(tx, ty, b1x, b1y, mx, my, color);
  g.fillTriangle(tx, ty, mx, my, b2x, b2y, color);
}

void updateNearestFieldsScreen(bool force) {
  if (currentScreen != NEAREST_FIELDS_SCREEN) return;
  const unsigned long now = millis();
  if (!force && now - nfLastDrawMs < 1000UL) return;   // un ridisegno al secondo
  if (nfLastDrawMs == 0 || now - nfLastDrawMs > 3000UL) nfOffset = 0;   // pagina appena aperta: dai piu' vicini
  nfLastDrawMs = now;
  bool gpsValid = false;
  double lat = 0.0, lon = 0.0;
  float course = 0.0f;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    gpsValid = gpsDataValid; lat = gps_latitude; lon = gps_longitude; course = gps_course;
    xSemaphoreGive(sensorDataMutex);
  } else if (!force) return;
  if (!ensureNavigationSprite()) return;
  windDotShown = false;
  lgfx::LGFXBase &g = navigationSprite;
  const uint16_t bg = themeBg();
  const uint16_t rowBg = displayTheme == THEME_COLOR ? lgfx::color565(17, 17, 17) : bg;
  const uint16_t grey = themeColor(lgfx::color565(170, 170, 170));
  const uint16_t ring = themeColor(lgfx::color565(60, 60, 60));
  navigationSprite.fillSprite(bg);
  drawUiVlwSprite(navigationSprite, "NEAREST FIELDS", 180, 50, MC_DATUM, TFT_CYAN, bg, 0.62f);
  const bool nfPhone = phonePosFill(gpsValid, lat, lon);
  if (nfPhone) drawUiVlwSprite(navigationSprite, "POSITION FROM PHONE", 180, 74, MC_DATUM,
                               themeColor(TFT_DARKGREY), bg, 0.34f);
  if (!gpsValid) {
    navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
    navigationSprite.setTextSize((1.35f) * THEME_TEXT_SCALE, (1.35f) * THEME_TEXT_SCALE);
    navigationSprite.setTextDatum(MC_DATUM);
    navigationSprite.setTextColor(themeColor(TFT_RED), bg);
    navigationSprite.drawString("NO GPS FIX", 180, 180);
    for (int k = 0; k < 3; ++k) nfIdx[k] = -1;
  } else {
    nearestFieldsSearch(lat, lon);
    const uint16_t distColor[3] = { TFT_GREEN, TFT_YELLOW, TFT_ORANGE };   // 1o, 2o, dal 3o in poi
    for (int k = 0; k < 3; ++k) {
      const int i = nfIdx[k];
      if (i < 0) continue;
      const AirfieldRec &f = AIRFIELDS[i];
      const int y = NF_ROW_Y0 + k * NF_ROW_STEP;
      const bool isGoto = navGotoActive && navGotoIdx == i;
      g.fillRoundRect(NF_ROW_X, y, NF_ROW_W, NF_ROW_H, 10, rowBg);
      const int rank = nfOffset + k;   // 0 = il piu' vicino
      const uint16_t border = isGoto ? themeColor(TFT_ORANGE) : (rank == 0 ? themeVivid(TFT_GREEN) : ring);
      g.drawRoundRect(NF_ROW_X, y, NF_ROW_W, NF_ROW_H, 10, border);
      if (rank == 0 || isGoto) g.drawRoundRect(NF_ROW_X + 1, y + 1, NF_ROW_W - 2, NF_ROW_H - 2, 9, border);
      float rel = nfBearing[k] - course;
      while (rel < 0.0f) rel += 360.0f;
      while (rel >= 360.0f) rel -= 360.0f;
      nfDrawArrow(g, NF_ROW_X + 30, y + 32, rel, themeColor(TFT_WHITE), ring);
      drawUiVlwSprite(navigationSprite, f.name, NF_ROW_X + 60, y + 21, ML_DATUM, TFT_WHITE, rowBg, 0.56f);
      char buf[24];
      if (nfDistKm[k] < 100.0f) snprintf(buf, sizeof(buf), "%.1f km", nfDistKm[k]);
      else snprintf(buf, sizeof(buf), "%d km", (int)lroundf(nfDistKm[k]));
      drawUiVlwSprite(navigationSprite, buf, NF_ROW_X + NF_ROW_W - 12, y + 21, MR_DATUM, distColor[min(rank, 2)], rowBg, 0.56f);
      buf[0] = 0;
      if (f.rwy > 0) {
        const int a = f.rwy, b = f.rwy + 18;
        if (f.rwLenM > 0) snprintf(buf, sizeof(buf), "RWY %02d/%02d %u m", a, b, (unsigned)f.rwLenM);
        else snprintf(buf, sizeof(buf), "RWY %02d/%02d", a, b);
      } else if (f.rwLenM > 0) snprintf(buf, sizeof(buf), "RWY %u m", (unsigned)f.rwLenM);
      if (buf[0]) drawUiVlwSprite(navigationSprite, buf, NF_ROW_X + 60, y + 46, ML_DATUM, grey, rowBg, 0.42f);
      if (f.freqK > 0) {
        const unsigned long fk = 100000UL + f.freqK;
        snprintf(buf, sizeof(buf), "%lu.%03lu", fk / 1000UL, fk % 1000UL);
        drawUiVlwSprite(navigationSprite, buf, NF_ROW_X + NF_ROW_W - 12, y + 46, MR_DATUM, grey, rowBg, 0.42f);
      }
    }
  }
  // pulsante SEARCH (lente): apre la ricerca di un campo con la tastiera
  {
    const uint16_t yc = themeColor(TFT_YELLOW);
    g.drawRoundRect(NF_SEARCH_X, NF_SEARCH_Y, NF_SEARCH_W, NF_SEARCH_H, 8, yc);
    g.drawRoundRect(NF_SEARCH_X + 1, NF_SEARCH_Y + 1, NF_SEARCH_W - 2, NF_SEARCH_H - 2, 7, yc);
    const int lx = NF_SEARCH_X + 24, ly = NF_SEARCH_Y + NF_SEARCH_H / 2 - 2;
    g.drawCircle(lx, ly, 7, yc); g.drawCircle(lx, ly, 6, yc);
    g.drawWideLine(lx + 5, ly + 5, lx + 11, ly + 11, 2.2f, yc);
    drawUiVlwSprite(navigationSprite, "SEARCH", NF_SEARCH_X + 76, NF_SEARCH_Y + NF_SEARCH_H / 2 + 1, MC_DATUM, TFT_YELLOW, bg, 0.5f);
  }
  if (navGotoActive) {
    char t[32];
    snprintf(t, sizeof(t), "GO TO %.18s", AIRFIELDS[navGotoIdx].name);
    drawUiVlwSprite(navigationSprite, t, 180, 344, MC_DATUM, TFT_ORANGE, bg, 0.38f);
  }
  if (gpsValid && nfFound > 0) {   // posizione nell'elenco e frecce dello scorrimento, ai lati del pulsante
    char t[24];
    snprintf(t, sizeof(t), "%d-%d", nfOffset + 1, min(nfOffset + 3, nfFound));
    drawUiVlwSprite(navigationSprite, t, 94, NF_SEARCH_Y + NF_SEARCH_H / 2, MC_DATUM, grey, bg, 0.42f);
    const uint16_t ac = themeColor(TFT_CYAN);
    const int cy = NF_SEARCH_Y + NF_SEARCH_H / 2;
    if (nfOffset + 3 < nfFound) g.fillTriangle(250, cy + 5, 264, cy + 5, 257, cy - 6, ac);   // swipe in su: successivi
    if (nfOffset > 0) g.fillTriangle(272, cy - 5, 286, cy - 5, 279, cy + 6, ac);            // swipe in giu': precedenti
  }
  navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

// Swipe verticale sulla pagina: dir +1 = campi successivi (swipe in su), -1 = precedenti (swipe in giu').
void nearestFieldsScroll(int dir) {
  const int next = nfOffset + dir * 3;
  if (next < 0 || next >= nfFound) { playTone(600, 60); return; }   // inizio o fine dell'elenco
  nfOffset = next;
  playTone(1200, 50);
  updateNearestFieldsScreen(true);
}

// Tocco su una riga: GO TO verso quel campo e passaggio a GO HOME.
void openFieldSearch();
void nearestFieldsTap(int x, int y) {
  if (x >= NF_SEARCH_X - 8 && x <= NF_SEARCH_X + NF_SEARCH_W + 8 && y >= NF_SEARCH_Y - 6 && y <= NF_SEARCH_Y + NF_SEARCH_H + 10) {
    playTone(1500, 80);
    openFieldSearch();   // SEARCH: ricerca del campo con la tastiera
    return;
  }
  if (x < NF_ROW_X - 6 || x > NF_ROW_X + NF_ROW_W + 6) return;
  for (int k = 0; k < 3; ++k) {
    const int ry = NF_ROW_Y0 + k * NF_ROW_STEP;
    if (y >= ry - 3 && y <= ry + NF_ROW_H + 3 && nfIdx[k] >= 0) {
      navSetGoto(nfIdx[k]);
      bleQueuePrefs();   // slave: anche il master (e gli altri slave) vanno verso lo stesso campo
      playTone(1500, 80);
      currentScreen = GO_TO_HOME_SCREEN;
      for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == GO_TO_HOME_SCREEN) { currentScreenIndex = i; break; }
      saveScreenPreference();
      drawGoToHomeScreen();
      return;
    }
  }
}


// ========== RICERCA CAMPO (SEARCH in NEAREST FIELDS) ==========
// Casella con la lente e DEL in alto, 3 risultati, tastiera A-Z su 4 righe (sempre piu' corte per stare nel cerchio).
// Senza lettere: i 3 campi piu' vicini. Con lettere: i campi che le contengono in qualunque punto del nome,
// i piu' vicini per primi. Tocco su un risultato: GO TO (come NEAREST FIELDS). Swipe: torna a NEAREST FIELDS.
static char srQuery[12] = "";
static const int SR_MAX = 30;               // risultati tenuti (i piu' vicini), a pagine di 3 con lo swipe su/giu'
static int srAll[SR_MAX];
static int srAllN = 0;
static int srOff = 0;                       // primo risultato mostrato
static int srRes[3] = { -1, -1, -1 };
static float srResKm[3] = { 0, 0, 0 };
static int srMatches = 0;
static unsigned long srLastDrawMs = 0;
static const char *const SR_KEYS[4] = { "ABCDEFGH", "IJKLMNO", "PQRSTU", "VWXYZ" };
static const int SR_KEY_Y[4] = { 173, 213, 253, 293 };   // bordo alto dei tasti (alti 34)
static const int SR_BOX_X = 64, SR_BOX_Y = 36, SR_BOX_W = 170, SR_BOX_H = 36;
static const int SR_DEL_X = 240, SR_DEL_W = 56;
static const int SR_RES_Y0 = 96, SR_RES_STEP = 28;
static int srKeyPitch(int row) { return row == 3 ? 40 : 42; }
static int srKeyX0(int row) { return 180 - (int)strlen(SR_KEYS[row]) * srKeyPitch(row) / 2; }

// Sigla del campo (colonna code del file .cup); vuota per l'elenco compilato nel firmware.
static const char *airfieldCode(int i);
static void fieldSearchRun(double lat, double lon, bool gpsOk) {
  static float best[SR_MAX];
  static int bi[SR_MAX];
  for (int k = 0; k < SR_MAX; ++k) { best[k] = 1e9f; bi[k] = -1; }
  srMatches = 0;
  const double kLon = cos(lat * DEG_TO_RAD);
  char up[40];
  for (int i = 0; i < AIRFIELD_COUNT; ++i) {
    if (srQuery[0]) {
      int j = 0;
      for (; AIRFIELDS[i].name[j] && j < 39; ++j) up[j] = (char)toupper((unsigned char)AIRFIELDS[i].name[j]);
      up[j] = '\0';
      bool hit = strstr(up, srQuery) != nullptr;
      if (!hit) {   // anche nella sigla (es. LIML)
        const char *cd = airfieldCode(i);
        for (j = 0; cd[j] && j < 15; ++j) up[j] = (char)toupper((unsigned char)cd[j]);
        up[j] = '\0';
        hit = j > 0 && strstr(up, srQuery) != nullptr;
      }
      if (!hit) continue;
    }
    ++srMatches;
    float d2 = (float)i;   // senza GPS: ordine dell'elenco
    if (gpsOk) {
      const float dy = (float)(AIRFIELDS[i].lat5 / 100000.0 - lat);
      const float dx = (float)((AIRFIELDS[i].lon5 / 100000.0 - lon) * kLon);
      d2 = dx * dx + dy * dy;
    }
    if (d2 >= best[SR_MAX - 1]) continue;
    int k = SR_MAX - 1;
    while (k > 0 && d2 < best[k - 1]) { best[k] = best[k - 1]; bi[k] = bi[k - 1]; --k; }
    best[k] = d2; bi[k] = i;
  }
  srAllN = 0;
  for (int k = 0; k < SR_MAX; ++k) if (bi[k] >= 0) srAll[srAllN++] = bi[k];
  if (srOff >= srAllN) srOff = srAllN > 0 ? ((srAllN - 1) / 3) * 3 : 0;
  for (int k = 0; k < 3; ++k) {
    const int idx = srOff + k < srAllN ? srAll[srOff + k] : -1;
    bi[k] = idx;
    srRes[k] = bi[k];
    srResKm[k] = -1.0f;
    if (bi[k] >= 0 && gpsOk) {
      const double la1 = lat * DEG_TO_RAD, lo1 = lon * DEG_TO_RAD;
      const double la2 = AIRFIELDS[bi[k]].lat5 / 100000.0 * DEG_TO_RAD, lo2 = AIRFIELDS[bi[k]].lon5 / 100000.0 * DEG_TO_RAD;
      srResKm[k] = (float)(lubberAngDist(la1, lo1, la2, lo2) * 6371.0);
    }
  }
}

void drawFieldSearchScreen() {
  srLastDrawMs = millis();
  if (!ensureNavigationSprite()) return;
  ForceColorThemeScope forceColor;
  LGFX_Sprite &g = navigationSprite;
  const uint16_t bg = TFT_BLACK, blue = lgfx::color565(22, 133, 255), org = TFT_ORANGE;
  g.fillSprite(bg);
  bool gpsOk = false; double lat = 0, lon = 0;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    gpsOk = gpsDataValid; lat = gps_latitude; lon = gps_longitude;
    xSemaphoreGive(sensorDataMutex);
  }
  fieldSearchRun(lat, lon, gpsOk);
  // casella di ricerca con la lente, e DEL
  g.drawRoundRect(SR_BOX_X, SR_BOX_Y, SR_BOX_W, SR_BOX_H, 8, blue);
  g.drawRoundRect(SR_BOX_X + 1, SR_BOX_Y + 1, SR_BOX_W - 2, SR_BOX_H - 2, 7, blue);
  const int lx = SR_BOX_X + 20, ly = SR_BOX_Y + SR_BOX_H / 2 - 2;
  g.drawCircle(lx, ly, 6, blue); g.drawCircle(lx, ly, 5, blue);
  g.drawWideLine(lx + 4, ly + 4, lx + 9, ly + 9, 2.0f, blue);
  char q[16];
  snprintf(q, sizeof(q), "%s_", srQuery);
  drawUiVlwSprite(g, q, SR_BOX_X + 38, SR_BOX_Y + SR_BOX_H / 2 + 1, ML_DATUM, TFT_WHITE, bg, 0.62f);
  g.drawRoundRect(SR_DEL_X, SR_BOX_Y, SR_DEL_W, SR_BOX_H, 8, org);
  g.drawRoundRect(SR_DEL_X + 1, SR_BOX_Y + 1, SR_DEL_W - 2, SR_BOX_H - 2, 7, org);
  drawUiVlwSprite(g, "DEL", SR_DEL_X + SR_DEL_W / 2, SR_BOX_Y + SR_BOX_H / 2 + 1, MC_DATUM, TFT_ORANGE, bg, 0.5f);
  // risultati
  for (int k = 0; k < 3; ++k) {
    const int y = SR_RES_Y0 + k * SR_RES_STEP;
    if (srRes[k] < 0) {
      if (k == 0) drawUiVlwSprite(g, srQuery[0] ? "No field found" : (gpsOk ? "" : "NO GPS FIX"), 180, y, MC_DATUM,
                                  lgfx::color565(170, 170, 170), bg, 0.48f);
      continue;
    }
    const uint16_t rb = k == 0 ? lgfx::color565(17, 34, 51) : bg;
    g.fillRoundRect(40, y - 13, 280, 26, 4, rb);
    char nm[20];
    snprintf(nm, sizeof(nm), "%.17s", AIRFIELDS[srRes[k]].name);
    for (char *c = nm; *c; ++c) *c = (char)toupper((unsigned char)*c);
    drawUiVlwSprite(g, nm, 50, y + 1, ML_DATUM, TFT_WHITE, rb, 0.48f);
    if (srResKm[k] >= 0.0f) {
      char d[16];
      if (srResKm[k] < 100.0f) snprintf(d, sizeof(d), "%.1f km", srResKm[k]); else snprintf(d, sizeof(d), "%d km", (int)lroundf(srResKm[k]));
      drawUiVlwSprite(g, d, 312, y + 1, MR_DATUM, lgfx::color565(136, 221, 136), rb, 0.45f);
    }
  }
  // altri risultati: frecce a destra (swipe su = successivi, giu' = precedenti) e numero
  if (srAllN > 3) {
    const uint16_t ac = lgfx::color565(136, 221, 136), ad = lgfx::color565(60, 60, 60);
    const int ax = 334;
    const uint16_t cu = srOff > 0 ? ac : ad, cdn = srOff + 3 < srAllN ? ac : ad;
    g.fillTriangle(ax - 7, SR_RES_Y0 + 4, ax + 7, SR_RES_Y0 + 4, ax, SR_RES_Y0 - 8, cu);
    g.fillTriangle(ax - 7, SR_RES_Y0 + 2 * SR_RES_STEP - 4, ax + 7, SR_RES_Y0 + 2 * SR_RES_STEP - 4, ax, SR_RES_Y0 + 2 * SR_RES_STEP + 8, cdn);
    char pg[8];
    snprintf(pg, sizeof(pg), "%d", min(srAllN, srOff + 3));
    drawUiVlwSprite(g, pg, ax, SR_RES_Y0 + SR_RES_STEP, MC_DATUM, lgfx::color565(170, 170, 170), bg, 0.36f);
  }
  // tastiera
  const uint16_t kf = lgfx::color565(34, 34, 34), kb = lgfx::color565(102, 102, 102);
  for (int r = 0; r < 4; ++r) {
    const int n = (int)strlen(SR_KEYS[r]), p = srKeyPitch(r), x0 = srKeyX0(r);
    for (int k = 0; k < n; ++k) {
      const int x = x0 + k * p + 1, y = SR_KEY_Y[r];
      g.fillRoundRect(x, y, 38, 34, 6, kf);
      g.drawRoundRect(x, y, 38, 34, 6, kb);
      char ch[2] = { SR_KEYS[r][k], 0 };
      drawUiVlwSprite(g, ch, x + 19, y + 18, MC_DATUM, TFT_WHITE, kf, 0.6f);
    }
  }
  g.pushSprite(0, 0); drawPageLockOverlay();
}

void openFieldSearch() {
  srQuery[0] = '\0';
  srOff = 0;
  currentScreen = FIELD_SEARCH_SCREEN;
  drawFieldSearchScreen();
}
// distanze aggiornate ogni 2 secondi
void updateFieldSearchScreen() {
  if (currentScreen != FIELD_SEARCH_SCREEN) return;
  if (millis() - srLastDrawMs >= 2000UL) drawFieldSearchScreen();
}
void fieldSearchTouchRelease(bool tapValid, int diffX, int diffY, int x, int y) {
  if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {   // swipe: torna a NEAREST FIELDS
    currentScreen = NEAREST_FIELDS_SCREEN;
    updateNearestFieldsScreen(true);
    playTone(1500, 80);
    return;
  }
  if (abs(diffY) > SWIPE_MIN_Y_PX && abs(diffY) > abs(diffX)) {   // swipe su / giu': altri 3 risultati
    const int prev = srOff;
    if (diffY < 0 && srOff + 3 < srAllN) srOff += 3;
    else if (diffY > 0 && srOff > 0) srOff -= 3;
    playTone(srOff != prev ? 1200 : 600, 30);
    if (srOff != prev) drawFieldSearchScreen();
    return;
  }
  if (!tapValid) return;
  const size_t len = strlen(srQuery);
  if (x >= SR_DEL_X - 6 && x <= SR_DEL_X + SR_DEL_W + 8 && y >= SR_BOX_Y - 8 && y <= SR_BOX_Y + SR_BOX_H + 6) {   // DEL
    if (len > 0) { srQuery[len - 1] = '\0'; srOff = 0; playTone(1200, 30); } else playTone(600, 40);
    drawFieldSearchScreen();
    return;
  }
  for (int r = 0; r < 4; ++r) {   // tasti
    if (y < SR_KEY_Y[r] - 3 || y > SR_KEY_Y[r] + 37) continue;
    const int p = srKeyPitch(r), x0 = srKeyX0(r), n = (int)strlen(SR_KEYS[r]);
    const int k = (x - x0) / p;
    if (x < x0 || k < 0 || k >= n) return;
    if (len < sizeof(srQuery) - 1) { srQuery[len] = SR_KEYS[r][k]; srQuery[len + 1] = '\0'; srOff = 0; playTone(1200, 30); }
    else playTone(600, 40);
    drawFieldSearchScreen();
    return;
  }
  for (int k = 0; k < 3; ++k) {   // risultati: GO TO
    const int ry = SR_RES_Y0 + k * SR_RES_STEP;
    if (y >= ry - 14 && y <= ry + 13 && x >= 34 && x <= 326 && srRes[k] >= 0) {
      navSetGoto(srRes[k]);
      bleQueuePrefs();   // slave: anche il master (e gli altri slave) vanno verso lo stesso campo
      playTone(1500, 80);
      currentScreen = GO_TO_HOME_SCREEN;
      for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == GO_TO_HOME_SCREEN) { currentScreenIndex = i; break; }
      saveScreenPreference();
      drawGoToHomeScreen();
      return;
    }
  }
}

// ========== FIELDS 30 km: i campi di volo attorno all'aereo, rotta in alto ==========
// Nel giro tra COMPASS e GO TO HOME. Aereo al centro, rotta in alto, anelli a 10 / 20 / 30 km (nessuno zoom).
// Ogni campo entro 30 km e' un trattino orientato come la pista (cerchio in piu' se la pista e' >= 1200 m);
// i 3 piu' vicini in verde, giallo e arancione come in NEAREST FIELDS, gli altri in grigio chiaro.
// Home: casetta verde. In basso il campo piu' vicino con distanza e rotta, e quanti campi ci sono.
// Tocco su un campo: GO TO verso quel campo (pagina GO TO HOME).
// Zoom 10 / 20 / 30 km con lo swipe su (piu' vicino) / giu' (piu' lontano); la scelta resta dopo il riavvio
// (NVS "pages", chiave "frzoom": non toccata da Reset preferences). Linee dei CTR dal file spazi aerei.
static int frRangeKm = 30;
static float FR_RANGE_KM = 30.0f;
static float FR_PX_PER_KM = 165.0f / 30.0f;          // raggio scelto = 165 px dal centro
// Autozoom: con il campo piu' vicino entro 5 km la scala scende a 8 / 4 / 2 km (l'aereo resta uguale);
// oltre 6 km torna allo zoom scelto. Uno swipe durante l'autozoom lo spegne fino alla prossima uscita oltre 6 km.
static float frAutoKm = 0.0f;            // 0 = autozoom spento, altrimenti 30 / 20 / 10 / 5 / 2 / 1.5 km
static float frAutoBand = 0.0f;          // scala che l'autozoom vorrebbe: quando cambia, l'autozoom riparte anche dopo uno swipe
static bool frAutoSuppressed = false;
static void frApplyRange(int km) {
  frRangeKm = (km == 10 || km == 20) ? km : 30;
  FR_RANGE_KM = (float)frRangeKm;
  FR_PX_PER_KM = 165.0f / FR_RANGE_KM;
}
void loadFieldsRadarZoom() {
  int km = 30;
  if (preferences.begin("pages", true)) { km = preferences.getUChar("frzoom", 30); preferences.end(); }
  frApplyRange(km);
}
void aspDrawCtrOutlines(lgfx::LGFXBase &g, double lat, double lon, float course, float rangeKm, float pxPerKm);
static const int FR_MAX = 48;
static int frIdx[FR_MAX];
static int16_t frX[FR_MAX], frY[FR_MAX];
static int frCount = 0;
static unsigned long frLastDrawMs = 0;

// Posizione sullo schermo (rotta in alto) di un punto a distanza d (km) e rotta vera brg (gradi)
static void frProject(float dKm, float brgDeg, float course, int &x, int &y) {
  const float rel = (brgDeg - course) * DEG_TO_RAD;
  x = 180 + (int)lroundf(sinf(rel) * dKm * FR_PX_PER_KM);
  y = 180 - (int)lroundf(cosf(rel) * dKm * FR_PX_PER_KM);
}

static void frDrawPlane(lgfx::LGFXBase &g, uint16_t c, uint16_t edge) {
  // aereo stilizzato, muso in alto
  g.fillTriangle(180, 160, 175, 182, 185, 182, edge);
  g.fillTriangle(180, 162, 176, 181, 184, 181, c);
  g.fillRect(162, 176, 36, 6, edge); g.fillRect(163, 177, 34, 4, c);     // ali
  g.fillRect(177, 180, 6, 16, edge); g.fillRect(178, 180, 4, 15, c);     // fusoliera
  g.fillRect(171, 194, 18, 5, edge); g.fillRect(172, 195, 16, 3, c);     // piani di coda
}

// Colori fissi della pagina (come una carta VFR), uguali in tutti i temi
#define FR_C_BG    lgfx::color565(246, 236, 230)   // fondo crema
#define FR_C_INK   lgfx::color565(30, 42, 85)      // nomi e titolo, blu scuro
#define FR_C_BLUE  lgfx::color565(47, 109, 181)    // simbolo dei campi
#define FR_C_RING  lgfx::color565(154, 167, 194)   // anelli di distanza
#define FR_C_RINGT lgfx::color565(107, 120, 150)
#define FR_C_N     lgfx::color565(192, 57, 43)     // N rossa
#define FR_C_GREEN lgfx::color565(24, 163, 58)
#define FR_C_YEL   lgfx::color565(217, 164, 0)
#define FR_C_ORG   lgfx::color565(224, 112, 0)

// Testo senza riquadro di fondo (si fonde con il colore che c'e' sotto: crema o rosa del CTR)
static int frText(const char *t, int x, int y, decltype(MC_DATUM) datum, uint16_t fg, float scale) {
  bool loaded = vlwSet(navigationSprite, UiFont32);
  if (loaded) navigationSprite.setTextSize(scale, scale);
  else { navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); navigationSprite.setTextSize(scale * 1.5f, scale * 1.5f); }
  navigationSprite.setTextDatum(datum);
  navigationSprite.setTextColor(fg);
  const int w = navigationSprite.textWidth(t);
  navigationSprite.drawString(t, x, y);
  if (loaded) vlwRelease(navigationSprite);
  return w;
}

// ---- Vista della pista (entro 2 km dal campo piu' vicino): pista vista dall'alto in obliquo, come un disegno ----
// L'aereo sparisce; la pista si ingrandisce avvicinandosi e gira con la rotta. Testata vicina = quella da cui arrivi.
// Pista piu' larga del vero (lunghezza / 6) per essere leggibile. In alto nome del campo e frequenza; N E S W sul bordo.
static bool frRwyView = false;          // vista pista in corso
static bool frInfoOpen = false;         // pagina informazioni del campo (tocco al centro)
static int frFocusIdx = -1;             // campo di riferimento: GO TO se attivo, altrimenti il piu' vicino
static const int FR_HOME_X = 136, FR_HOME_Y = 300, FR_HOME_W = 88, FR_HOME_H = 30;   // pulsante HOME (GO TO attivo)
static int frRwyLblN = 0, frRwyLblX[2], frRwyLblY[2];   // riquadri dei numeri pista (le lettere N E S W si nascondono sotto)
static bool frRwySuppressed = false;    // spenta con uno swipe fino a quando ci si allontana oltre 2.5 km
// cifre a 7 segmenti nel piano della pista: segmenti a,b,c,d,e,f,g in un riquadro 0..1 x 0..1 (y verso il basso)
static const uint8_t FR_SEG[10] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };
static const float FR_SEG_XY[7][4] = { {0,0,1,0}, {1,0,1,0.5f}, {1,0.5f,1,1}, {0,1,1,1}, {0,0.5f,0,1}, {0,0,0,0.5f}, {0,0.5f,1,0.5f} };

struct FrCam { float px, py, fwx, fwy, rtx, rty, h, cth, sth, F, y0; };
static FrCam frCam;   // camera globale: niente struct nei parametri (prototipi automatici di Arduino)
// punto del suolo (m est, m nord rispetto al centro pista) -> schermo; false se dietro la camera
static bool frCamProj(float x, float y, float &sx, float &sy) {
  const FrCam &c = frCam;
  const float dx = x - c.px, dy = y - c.py;
  const float X = dx * c.rtx + dy * c.rty, Yf = dx * c.fwx + dy * c.fwy;
  const float zc = Yf * c.cth + c.h * c.sth;
  if (zc < 5.0f) return false;
  const float yc = Yf * c.sth - c.h * c.cth;
  sx = 180.0f + c.F * X / zc;
  sy = c.y0 - c.F * yc / zc;     // centro della vista: un po' in basso per il cielo e il nome in alto
  return fabsf(sx) < 4000 && fabsf(sy) < 4000;
}


// Pagina informazioni del campo di riferimento: nome, pista, lunghezza, orientamenti, frequenza,
// distanza e rotta, coordinate. Colori della carta. Un tocco chiude.
// Larghezza di un testo con il carattere della pagina, senza disegnarlo
static int frTextW(const char *t, float scale) {
  bool loaded = vlwSet(navigationSprite, UiFont32);
  if (loaded) navigationSprite.setTextSize(scale, scale);
  else { navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); navigationSprite.setTextSize(scale * 1.5f, scale * 1.5f); }
  const int w = navigationSprite.textWidth(t);
  if (loaded) vlwRelease(navigationSprite);
  return w;
}
// Scritta centrata ridotta quanto basta per stare in maxW pixel
static void frTextFit(const char *t, int x, int y, uint16_t fg, float scale, int maxW) {
  const int w = frTextW(t, scale);
  if (w > maxW && w > 0) scale *= (float)maxW / (float)w;
  frText(t, x, y, MC_DATUM, fg, scale);
}
// Larghezza disponibile dentro il display tondo all'altezza y, tenendo conto dell'altezza del carattere:
// si misura sul lato del testo piu' vicino al bordo, cosi' non escono nemmeno gli angoli delle lettere.
static int frCircleW(int y, float scale) {
  const float R = 172.0f;                    // raggio utile, un filo dentro il bordo
  const float h = 32.0f * scale;             // altezza del carattere alla scala data
  const float d1 = fabsf((float)y - h / 2 - 180.0f), d2 = fabsf((float)y + h / 2 - 180.0f);
  const float d = max(d1, d2);
  if (d >= R) return 0;
  return (int)(2.0f * sqrtf(R * R - d * d));
}
// Scritta centrata che si adatta da sola al cerchio: si rimpicciolisce quanto basta, non di piu'.
static void frTextFitCircle(const char *t, int x, int y, uint16_t fg, float scale) {
  const float s0 = scale;
  for (int it = 0; it < 4; ++it) {
    const int maxW = frCircleW(y, scale), w = frTextW(t, scale);
    if (w <= 0 || maxW <= 0) break;
    const float k = (float)maxW / (float)w;
    if (k >= 1.0f && scale >= s0 - 0.001f) break;        // ci sta gia' alla dimensione piena
    if (k >= 0.995f && k <= 1.005f) break;
    scale = min(s0, scale * k);
  }
  frText(t, x, y, MC_DATUM, fg, scale);
}
static void frDrawFieldInfo(int fi, double lat, double lon, float gsKmh) {
  lgfx::LGFXBase &g = navigationSprite;
  const AirfieldRec &f = AIRFIELDS[fi];
  navigationSprite.fillSprite(FR_C_BG);
  // Tutte le scritte +30 %. Nome con due o piu' parole su due righe ("MILANO" sopra, "MALPENSA" sotto);
  // ogni riga si riduce da sola se non sta nel cerchio.
  char nm[42]; int j = 0;
  for (; f.name[j] && j < 40; ++j) nm[j] = (char)toupper((unsigned char)f.name[j]);
  nm[j] = '\0';
  // Taglio nel separatore piu' vicino alla meta' del nome: le due righe restano equilibrate
  // ("SAN GIOVANNI TEATINO" -> "SAN GIOVANNI" / "TEATINO", non "SAN" / "GIOVANNI TEATINO").
  const int nlen = j;
  char *sp = nullptr; int best = 1 << 30;
  for (int i = 1; i < nlen - 1; ++i) {
    if (nm[i] != ' ' && nm[i] != '-' && nm[i] != '_' && nm[i] != '/') continue;
    const int d = abs(i - nlen / 2);
    if (d < best) { best = d; sp = nm + i; }
  }
  if (sp != nullptr) {
    *sp = '\0';
    const char *rest = sp + 1;
    while (*rest == ' ' || *rest == '-') ++rest;
    frTextFitCircle(nm, 180, 40, FR_C_INK, 0.96f);
    frTextFitCircle(rest, 180, 76, FR_C_INK, 0.96f);
  } else {
    frTextFitCircle(nm, 180, 58, FR_C_INK, 0.96f);
  }
  if (f.freqK > 0) {
    const unsigned long fk = 100000UL + f.freqK;
    char fq[16]; snprintf(fq, sizeof(fq), "%lu.%03lu", fk / 1000UL, fk % 1000UL);
    frText(fq, 180, 114, MC_DATUM, FR_C_BLUE, 0.96f);
  }
  const uint16_t sepC = (navGotoActive && navGotoIdx == fi) ? FR_C_ORG : FR_C_RING;   // arancione: destinazione GO TO
  g.drawFastHLine(50, 138, 260, sepC); g.drawFastHLine(50, 139, 260, sepC);
  const double kl = cos(lat * DEG_TO_RAD);
  const float dy = (float)((f.lat5 / 100000.0 - lat) * 111.2), dx = (float)((f.lon5 / 100000.0 - lon) * kl * 111.2);
  const float dKm = sqrtf(dx * dx + dy * dy);
  char v[32];
  // colonne strette al centro (etichetta a destra fino a x=166, valore da x=178): tutto dentro il cerchio
  int y = 170;
  auto row = [&](const char *label, const char *val) {
    frText(label, 166, y, MR_DATUM, FR_C_RINGT, 0.73f);
    frText(val, 178, y, ML_DATUM, FR_C_INK, 1.12f);
    y += 45;
  };
  if (f.rwy > 0) { snprintf(v, sizeof(v), "%02d/%02d", f.rwy, (f.rwy + 18 - 1) % 36 + 1); row("RWY", v); }
  if (f.rwLenM > 0) { snprintf(v, sizeof(v), "%u m", (unsigned)f.rwLenM); row("LENGTH", v); }
  snprintf(v, sizeof(v), dKm < 10.0f ? "%.1f km" : "%.0f km", dKm); row("DIST", v);
  // minuti all'arrivo con la velocita' al suolo attuale (sotto 20 km/h non ha senso)
  if (gsKmh >= 20.0f) {
    const int mins = (int)lroundf(dKm / gsKmh * 60.0f);
    if (mins < 100) snprintf(v, sizeof(v), "%d min", mins); else snprintf(v, sizeof(v), "%dh%02d", mins / 60, mins % 60);
  } else snprintf(v, sizeof(v), "-- min");
  row("ETA", v);
  frText("Tap to exit", 180, 345, MC_DATUM, FR_C_RINGT, 0.5f);
}

// Campo dell'elenco che corrisponde alla home (entro 1 km), -1 se la home non e' un campo dell'elenco.
// Ricalcolato solo quando la home cambia.
static int frHomeFieldIdx() {
  static double cLat = 1e9, cLon = 1e9;
  static int cIdx = -1;
  static uint32_t cGen = 0xFFFFFFFFu;
  if (!homePositionValid) return -1;
  if (homeLatitude != cLat || homeLongitude != cLon || cGen != gAirfieldsGen) {
    cLat = homeLatitude; cLon = homeLongitude; cIdx = -1; cGen = gAirfieldsGen;
    const double kl = cos(cLat * DEG_TO_RAD);
    float best = 1.0f;
    for (int i = 0; i < AIRFIELD_COUNT; ++i) {
      const float dy = (float)((AIRFIELDS[i].lat5 / 100000.0 - cLat) * 111.2);
      if (fabsf(dy) > best) continue;
      const float dx = (float)((AIRFIELDS[i].lon5 / 100000.0 - cLon) * kl * 111.2);
      const float d = sqrtf(dx * dx + dy * dy);
      if (d <= best) { best = d; cIdx = i; }
    }
  }
  return cIdx;
}

// Distanza (km) dal segmento della pista vera (dal centro se l'orientamento non e' noto)
static float frRwyDistKm = 0.0f;
static float frRunwayDist(int fi, double lat, double lon) {
  const AirfieldRec &f = AIRFIELDS[fi];
  const double kLon = cos(lat * DEG_TO_RAD);
  const float px = (float)((lon - f.lon5 / 100000.0) * kLon * 111.2), py = (float)((lat - f.lat5 / 100000.0) * 111.2);
  if (f.rwy == 0) return sqrtf(px * px + py * py);
  const float L = (f.rwLenM > 0 ? f.rwLenM : 600) / 1000.0f;
  const float hdg = airfieldRwyDeg(fi);
  const float ux = sinf(hdg * DEG_TO_RAD), uy = cosf(hdg * DEG_TO_RAD);
  const float a = constrain(px * ux + py * uy, -L / 2, L / 2);
  const float dx = px - a * ux, dy = py - a * uy;
  return sqrtf(dx * dx + dy * dy);
}

static void frDrawRunwayView(int fi, double lat, double lon, float course) {
  lgfx::LGFXBase &g = navigationSprite;
  const AirfieldRec &f = AIRFIELDS[fi];
  // Pista disegnata come nel disegno approvato: le piste lunghe (aeroporti) si accorciano a 1000 m,
  // ma la distanza dalla soglia resta quella vera (la pista cresce avvicinandosi, anche a Linate o Malpensa)
  const float LREAL = f.rwLenM > 0 ? (float)f.rwLenM : 600.0f;
  const float LENF = min(LREAL, 1000.0f);             // lunghezza usata per la scala (riempie il display)
  float WID = constrain(LENF / 6.0f, 60.0f, 220.0f);   // poi: larga 48 px come i riquadri dei numeri
  const float rw = airfieldRwyDeg(fi) * DEG_TO_RAD;   // direzione vera della pista (se il file la da')
  const float ux = sinf(rw), uy = cosf(rw);          // verso della pista (rwy)
  const float lx = uy, ly = -ux;                      // laterale
  // Vista fissa: la pista resta al centro dello schermo, ruota con la rotta (rotta in alto) e si ingrandisce
  // avvicinandosi, da qualsiasi direzione; non esce mai dal display.
  FrCam &c = frCam;
  const float DV = 1400.0f;                           // camera virtuale: sempre a 1400 m dal centro pista
  const float hd = course * DEG_TO_RAD;
  c.fwx = sinf(hd); c.fwy = cosf(hd); c.rtx = cosf(hd); c.rty = -sinf(hd);
  // Vista 2D dall'alto (come una carta), con la rotta in alto: niente prospettiva, niente cielo.
  const float th = 1.5708f;
  const float D = DV;
  c.px = 0; c.py = 0;
  c.h = D;
  c.cth = 0.0f; c.sth = 1.0f;
  c.y0 = 180.0f;   // pista, numeri, nome e frequenza centrati sul display
  // Scala fissa: la pista e' sempre della stessa grandezza, qualunque sia la rotta, la distanza
  // e la lunghezza vera della pista (mezza pista 135 px). Cosi' non si rimpicciolisce mai ruotando.
  {
    const float hlWant = 135.0f;
    c.F = hlWant * c.h / (LENF / 2);
  }
  (void)th;
  WID = 48.0f * c.h / c.F;   // larghezza disegnata: 48 px
  const float LEN = LENF;
  g.fillScreen(lgfx::color565(214, 203, 170));   // fondo sabbia: il piu' leggibile al sole
  // quadrilatero nel piano della pista: s lungo la pista (m dal centro, + verso rwy), t laterale (m)
  auto quad = [&](float s0, float s1, float t0, float t1, uint16_t col) {
    float x[4], y[4];
    const float ss[4] = { s0, s0, s1, s1 }, tt[4] = { t0, t1, t1, t0 };
    (void)x; (void)y;
    // coordinate camera degli angoli, ritaglio a zc >= 5 m, poi proiezione e ventaglio di triangoli
    float X[4], Z[4], Yc[4];
    for (int k = 0; k < 4; ++k) {
      const float dx = ux * ss[k] + lx * tt[k] - c.px, dy = uy * ss[k] + ly * tt[k] - c.py;
      const float yf = dx * c.fwx + dy * c.fwy;
      X[k] = dx * c.rtx + dy * c.rty;
      Z[k] = yf * c.cth + c.h * c.sth;
      Yc[k] = yf * c.sth - c.h * c.cth;
    }
    float PX[8], PY[8]; int np = 0;
    const float ZN = 5.0f;
    for (int k = 0; k < 4 && np < 7; ++k) {
      const int m = (k + 1) & 3;
      const bool inK = Z[k] >= ZN, inM = Z[m] >= ZN;
      if (inK) { PX[np] = 180.0f + c.F * X[k] / Z[k]; PY[np] = c.y0 - c.F * Yc[k] / Z[k]; ++np; }
      if (inK != inM) {
        const float t = (ZN - Z[k]) / (Z[m] - Z[k]);
        const float xi = X[k] + (X[m] - X[k]) * t, yi = Yc[k] + (Yc[m] - Yc[k]) * t;
        PX[np] = 180.0f + c.F * xi / ZN; PY[np] = c.y0 - c.F * yi / ZN; ++np;
      }
    }
    for (int k = 0; k < np; ++k) { PX[k] = constrain(PX[k], -2000.0f, 2400.0f); PY[k] = constrain(PY[k], -2000.0f, 2400.0f); }
    for (int k = 1; k + 1 < np; ++k) g.fillTriangle(PX[0], PY[0], PX[k], PY[k], PX[k + 1], PY[k + 1], col);
  };
  const uint16_t white = TFT_WHITE;
  quad(-LEN / 2 - 25, LEN / 2 + 25, -WID / 2 - 25, WID / 2 + 25, lgfx::color565(192, 180, 146));   // fascia attorno alla pista
  quad(-LEN / 2, LEN / 2, -WID / 2, WID / 2, lgfx::color565(22, 22, 24));                          // asfalto nero
  quad(-LEN / 2, LEN / 2, -WID / 2 + WID * 0.03f, -WID / 2 + WID * 0.06f, white);                  // bordi
  quad(-LEN / 2, LEN / 2, WID / 2 - WID * 0.06f, WID / 2 - WID * 0.03f, white);
  {   // asse tratteggiato per tutta la lunghezza (i numeri sono etichette fuori dalla pista)
    const float dash = LEN / 30.0f, step = LEN / 16.0f;
    const float sEnd = LEN / 2 - LEN * 0.08f;                                                    // fino alle zebrature
    const int nD = (int)floorf((2 * sEnd - dash) / step) + 1;                                    // trattini simmetrici
    const float s0 = -((nD - 1) * step + dash) / 2;
    for (int k = 0; k < nD; ++k) quad(s0 + k * step, s0 + k * step + dash, -WID * 0.015f, WID * 0.015f, white);
  }
  for (int e = -1; e <= 1; e += 2) {                                             // testate: 3 barre sottili per lato, staccate dai bordi
    const float s0 = e * (LEN / 2 - LEN * 0.01f), s1 = e * (LEN / 2 - LEN * 0.06f);
    const float bw = WID * 0.026f, step = (WID * 0.40f - bw - WID * 0.16f) / 2.0f;
    for (int sg = -1; sg <= 1; sg += 2)
      for (int k = 0; k < 3; ++k) {
        const float t = sg * (WID * 0.16f + k * step);
        quad(min(s0, s1), max(s0, s1), min(t, t + sg * bw), max(t, t + sg * bw), white);
      }
  }
  // numeri delle testate: riquadri bianchi grandi (come la larghezza della pista), sempre dritti, 34 px oltre
  // ogni testata. Alla testata "+u" si legge rwy+18, alla testata "-u" rwy.
  frRwyLblN = 0;
  if (f.rwy > 0) {
    const int nPlus = (f.rwy + 18 - 1) % 36 + 1, nMinus = f.rwy;
    float cxp, cyp;
    if (frCamProj(0.0f, 0.0f, cxp, cyp)) {
      const float inM = 38.0f * c.h / c.F;   // riquadri sull'asfalto, 38 px dentro la testata
      (void)cxp; (void)cyp;
      // Inclinazione della pista sullo schermo: il riquadro quadrato ci resta sempre parallelo,
      // il numero dentro invece sta sempre dritto, cosi' in finale si legge come su una pista vera.
      float rwyAng = 0.0f;
      {
        float ax0, ay0, ax1, ay1;
        if (frCamProj(-ux * LEN / 2, -uy * LEN / 2, ax0, ay0) && frCamProj(ux * LEN / 2, uy * LEN / 2, ax1, ay1)) {
          float vx = ax1 - ax0, vy = ay1 - ay0;
          const float vl = sqrtf(vx * vx + vy * vy);
          if (vl > 10.0f) {
            vx /= vl; vy /= vl;
            if (vx < -0.05f || (fabsf(vx) <= 0.05f && vy > 0)) { vx = -vx; vy = -vy; }
            rwyAng = atan2f(vy, vx) / DEG_TO_RAD;
          }
        }
      }
      static LGFX_Sprite rbox;
      static bool rboxReady = false, rboxFailed = false;
      const uint16_t KEYB = 0x0821;
      if (!rboxReady && !rboxFailed) {
        rbox.setPsram(psramFound()); rbox.setColorDepth(16);
        if (rbox.createSprite(68, 68) != nullptr) rboxReady = true; else rboxFailed = true;
      }
      for (int e = -1; e <= 1; e += 2) {
        const int num = e > 0 ? nPlus : nMinus;
        float sx, sy;
        if (!frCamProj(ux * e * (LEN / 2 - inM), uy * e * (LEN / 2 - inM), sx, sy)) continue;
        const int bx = (int)lroundf(sx), by = (int)lroundf(sy);
        char t[4]; snprintf(t, sizeof(t), "%02d", num);
        const int BS = 46;   // riquadro quadrato: funziona con la pista in qualsiasi direzione
        const uint16_t boxBlue = lgfx::color565(0, 70, 160);   // blu scuro: stacca dal bianco al sole
        if (rboxReady) {
          rbox.fillSprite(KEYB);
          rbox.fillRoundRect(11, 11, BS, BS, 8, TFT_WHITE);
          rbox.drawRoundRect(11, 11, BS, BS, 8, boxBlue);
          rbox.drawRoundRect(12, 12, BS - 2, BS - 2, 7, boxBlue);
          rbox.pushRotateZoom(&navigationSprite, bx, by, rwyAng, 1.0f, 1.0f, KEYB);
        } else {
          g.fillRoundRect(bx - BS / 2, by - BS / 2, BS, BS, 8, TFT_WHITE);
          g.drawRoundRect(bx - BS / 2, by - BS / 2, BS, BS, 8, boxBlue);
          g.drawRoundRect(bx - BS / 2 + 1, by - BS / 2 + 1, BS - 2, BS - 2, 7, boxBlue);
        }
        // numero il piu' grande che ci sta: 1 px dal riquadro, dentro il bordo da 2 px, sempre dritto
        float nsc = 0.8f;
        {
          const bool loaded = vlwSet(navigationSprite, UiFont32);
          if (loaded) {
            navigationSprite.setTextSize(1.0f, 1.0f);
            const int w1 = navigationSprite.textWidth(t), h1 = navigationSprite.fontHeight();
            vlwRelease(navigationSprite);
            const float sw = w1 > 0 ? (float)(BS - 6) / (float)w1 : 0.8f;             // 46 - 2*2 bordo - 2*1
            const float sh = h1 > 0 ? (float)(BS - 6) / (0.75f * (float)h1) : 0.8f;   // le cifre non hanno discendenti
            nsc = min(sw, sh);
          }
        }
        frText(t, bx, by + 1, MC_DATUM, TFT_BLACK, nsc);
        if (frRwyLblN < 2) { frRwyLblX[frRwyLblN] = bx; frRwyLblY[frRwyLblN] = by; ++frRwyLblN; }
      }
    }
  }
  // Nome da una parte della pista e frequenza dall'altra, girati come la pista e sempre leggibili
  // da sinistra a destra. Nome troppo lungo per la pista: si accorcia l'ultima parola alla sola
  // iniziale (CORNEGLIANO LAUDENSE -> CORNEGLIANO L.).
  {
    char nm[26]; int j = 0;
    for (; f.name[j] && j < 24; ++j) nm[j] = (char)toupper((unsigned char)f.name[j]);
    nm[j] = '\0';
    char fqs[12]; fqs[0] = '\0';
    if (f.freqK > 0) { const unsigned long fk = 100000UL + f.freqK; snprintf(fqs, sizeof(fqs), "%lu.%03lu", fk / 1000UL, fk % 1000UL); }
    float ax0, ay0, ax1, ay1;
    if (frCamProj(-ux * LEN / 2, -uy * LEN / 2, ax0, ay0) && frCamProj(ux * LEN / 2, uy * LEN / 2, ax1, ay1)) {
      float vx = ax1 - ax0, vy = ay1 - ay0;
      const float vl = sqrtf(vx * vx + vy * vy);
      if (vl > 10.0f) {
        vx /= vl; vy /= vl;
        if (vx < -0.05f || (fabsf(vx) <= 0.05f && vy > 0)) { vx = -vx; vy = -vy; }
        static LGFX_Sprite lbl;
        static bool lblReady = false, lblFailed = false;
        const uint16_t KEYC = 0x0821;
        if (!lblReady && !lblFailed) {
          lbl.setPsram(psramFound()); lbl.setColorDepth(16);
          if (lbl.createSprite(320, 40) != nullptr) lblReady = true; else lblFailed = true;
        }
        if (lblReady) {
          const float avail = min(300.0f, max(90.0f, vl));
          if (vlwSet(lbl, UiFont32)) {
            lbl.setTextSize(0.66f, 0.66f);
            for (int guard = 0; guard < 4 && lbl.textWidth(nm) > avail; ++guard) {
              char *sp = strrchr(nm, ' ');
              if (sp == nullptr || sp[1] == '\0' || sp == nm) break;
              if (sp[2] == '.' && sp[3] == '\0') {          // ultima parola gia' ridotta: tocca a quella prima
                *sp = '\0';
                char *sp2 = strrchr(nm, ' ');
                *sp = ' ';
                if (sp2 == nullptr || sp2 == nm) break;
                sp = sp2;
              }
              sp[2] = '.'; sp[3] = '\0';
            }
            vlwRelease(lbl);
          }
          const float side = WID / 2 * c.F / c.h + 16.0f;   // px dal centro pista
          for (int k = 0; k < 2; ++k) {
            const char *txt = k == 0 ? nm : fqs;
            if (txt[0] == '\0') continue;
            const int sg = k == 0 ? +1 : -1;               // nome da una parte, frequenza dall'altra
            lbl.fillSprite(KEYC);
            if (vlwSet(lbl, UiFont32)) {
              float sc = 0.66f;   // nome e frequenza +20 %
              lbl.setTextSize(sc, sc);
              const int w = lbl.textWidth(txt);
              if (w > 300 && w > 0) { sc *= 300.0f / w; lbl.setTextSize(sc, sc); }
              lbl.setTextDatum(MC_DATUM);
              lbl.setTextColor(TFT_BLACK, lgfx::color565(214, 203, 170));   // nero sul fondo sabbia
              lbl.drawString(txt, 160, 20);
              vlwRelease(lbl);
            }
            const float cxl = (ax0 + ax1) / 2 + sg * vy * side, cyl = (ay0 + ay1) / 2 - sg * vx * side;
            lbl.pushRotateZoom(&navigationSprite, cxl, cyl, atan2f(vy, vx) / DEG_TO_RAD, 1.0f, 1.0f, KEYC);
          }
        }
      }
    }
  }
  // Aereo giallo con bordino nero (la stessa sagoma della bussola) sul bordo, nella direzione in cui
  // ci troviamo rispetto alla pista: in finale resta dietro la testata, in sottovento si sposta di lato.
  float frPlaneX = -1000.0f, frPlaneY = -1000.0f;
  {
    const double kL = cos(lat * DEG_TO_RAD);
    const float ax = (float)((lon - f.lon5 / 100000.0) * kL * 111200.0);
    const float ay = (float)((lat - f.lat5 / 100000.0) * 111200.0);
    if (ax * ax + ay * ay > 1.0f) {
      const float rr = (atan2f(ax, ay) / DEG_TO_RAD - course) * DEG_TO_RAD;   // vista con la rotta in alto
      frPlaneX = 180.0f + 154.0f * sinf(rr);   // sagoma a 2 px dal bordo del display
      frPlaneY = 180.0f - 154.0f * cosf(rr);
      static LGFX_Sprite frPlaneSpr;
      static bool sprReady = false, sprFailed = false;
      const uint16_t KEYP = 0xF81F;
      if (!sprReady && !sprFailed) {
        frPlaneSpr.setPsram(psramFound()); frPlaneSpr.setColorDepth(16);
        if (frPlaneSpr.createSprite(144, 140) != nullptr) {
          frPlaneSpr.fillSprite(KEYP);
          LGFX_Sprite tmp;                                  // sagoma nera ingrandita: fa da contorno
          tmp.setPsram(psramFound()); tmp.setColorDepth(16);
          if (tmp.createSprite(120, 116) != nullptr) {
            tmp.fillSprite(KEYP);
            drawAirplaneSprite(tmp, 60, 58, TFT_BLACK);
            tmp.pushRotateZoom(&frPlaneSpr, 72, 70, 0.0f, 1.10f, 1.10f, KEYP);   // bordino nero sottile
            tmp.deleteSprite();
          }
          drawAirplaneSprite(frPlaneSpr, 72, 70, lgfx::color565(175, 0, 120));   // magenta
          sprReady = true;
        } else sprFailed = true;
      }
      if (sprReady) frPlaneSpr.pushRotateZoomWithAA(&navigationSprite, frPlaneX, frPlaneY, 0.0f, 0.43f, 0.43f, KEYP);
      else { g.fillCircle(frPlaneX, frPlaneY, 14, lgfx::color565(175, 0, 120)); g.drawCircle(frPlaneX, frPlaneY, 14, TFT_BLACK); }
    }
  }
  // Bussola intera sul bordo come GO TO HOME (gira con la rotta): trattini ogni 10 gradi, lunghi ogni 30,
  // numeri e E S W neri, N arancione. Numeri e lettere sotto un riquadro della pista non si scrivono.
  {
    const uint16_t bgG = lgfx::color565(214, 203, 170);
    for (int angle = 0; angle < 360; angle += 10) {
      const float rad = (angle - course - 90.0f) * DEG_TO_RAD;
      const float co = cosf(rad), si = sinf(rad);
      const bool major = angle % 30 == 0;
      const float rIn = major ? 155.0f : 165.0f;
      const float tkx = 180 + 165.0f * co, tky = 180 + 165.0f * si;
      if (sqrtf((tkx - frPlaneX) * (tkx - frPlaneX) + (tky - frPlaneY) * (tky - frPlaneY)) < 30.0f) continue;   // sotto l'aereo
      g.drawWideLine(180 + rIn * co, 180 + rIn * si, 180 + 175 * co, 180 + 175 * si, major ? 2.0f : 1.5f, TFT_BLACK);
      if (!major) continue;
      const int nx = 180 + (int)lroundf(145.0f * co), ny = 180 + (int)lroundf(145.0f * si);
      bool hidden = false;
      for (int k = 0; k < frRwyLblN; ++k)
        if (abs(nx - frRwyLblX[k]) < 33 + 10 && abs(ny - frRwyLblY[k]) < 33 + 10) hidden = true;
      if (sqrtf((nx - frPlaneX) * (nx - frPlaneX) + (ny - frPlaneY) * (ny - frPlaneY)) < 36.0f) hidden = true;
      if (hidden) continue;
      if (angle % 90 == 0) {
        navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
        navigationSprite.setTextSize(1.0f, 1.0f);
        navigationSprite.setTextDatum(MC_DATUM);
        navigationSprite.setTextColor(angle == 0 ? lgfx::color565(190, 25, 25) : TFT_BLACK);   // N rosso scuro
        navigationSprite.drawString(angle == 0 ? "N" : (angle == 90 ? "E" : (angle == 180 ? "S" : "W")), nx, ny);
      } else {
        char b[4]; snprintf(b, sizeof(b), "%d", angle / 10);
        drawNumericVlwSprite(navigationSprite, b, nx, ny, MC_DATUM, TFT_BLACK, bgG, 0.34f);
      }
    }
  }
}

void updateFieldsRadarScreen(bool force) {
  static unsigned long frSeenMs = 0;   // pagina lasciata e poi ripresa: si riparte dalla mappa
  if (millis() - frSeenMs > 3000UL) frInfoOpen = false;
  frSeenMs = millis();
  if (currentScreen != FIELDS_RADAR_SCREEN) return;
  windDotShown = false;   // riempito solo se il pallino del vento viene disegnato
  const unsigned long now = millis();
  // Vista della pista: si alterna con GO TO HOME (bersaglio e pista della meta): 5 s la pista, 2 s GO TO HOME
  static bool frGoHomeShown = false;
  if (gAltBorrow == 0 && frRwyView && !frInfoOpen && ((now % 7000UL) >= 5000UL)) {
    if (!frGoHomeShown) { frGoHomeShown = true; goToHomeNeedsRedraw = true; }
    gAltBorrow = 1;
    currentScreen = GO_TO_HOME_SCREEN;    // la pagina GO TO HOME disegna solo se e' quella corrente
    updateGoToHomeScreen();
    currentScreen = FIELDS_RADAR_SCREEN;
    gAltBorrow = 0;
    return;
  }
  if (frGoHomeShown) { frGoHomeShown = false; force = true; }
  if (!force && now - frLastDrawMs < 1000UL) return;   // un ridisegno al secondo
  frLastDrawMs = now;
  bool gpsValid = false;
  double lat = 0.0, lon = 0.0;
  float course = 0.0f, gsKmh = 0.0f;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    gpsValid = gpsDataValid; lat = gps_latitude; lon = gps_longitude; course = gps_course; gsKmh = gps_ground_speed_kmh;
    xSemaphoreGive(sensorDataMutex);
  } else if (!force) return;
  if (!isfinite(course)) course = 0.0f;
  if (!isfinite(gsKmh)) gsKmh = 0.0f;
  // La posizione dal telefono si prende PRIMA di ogni altra cosa. Qui sotto si cercano il campo
  // piu' vicino, la scala automatica e i contorni degli spazi aerei, e tutte e tre guardano
  // gpsValid: finche' questa riga stava piu' in basso, quei tre pezzi vedevano ancora "niente
  // fix" e sparivano, mentre il resto della pagina si disegnava regolarmente con la posizione
  // del telefono. Da fuori sembrava che gli spazi aerei comparissero solo con i satelliti.
  const bool frPhone = phonePosFill(gpsValid, lat, lon);
  if (!ensureNavigationSprite()) return;
  ForceColorThemeScope forceColor;   // colori della carta in tutti i temi
  lgfx::LGFXBase &g = navigationSprite;
  navigationSprite.fillSprite(FR_C_BG);
  // Campo piu' vicino in assoluto (per l'autozoom)
  float nearKm = 1e9f;
  int nearIdx = -1;
  if (gpsValid) {
    const double kl = cos(lat * DEG_TO_RAD);
    for (int i = 0; i < AIRFIELD_COUNT; ++i) {
      const float dy = (float)((AIRFIELDS[i].lat5 / 100000.0 - lat) * 111.2);
      if (fabsf(dy) > 31.0f) continue;   // anche oltre i 5 km: serve all'autozoom 30 / 20 / 10
      const float dx = (float)((AIRFIELDS[i].lon5 / 100000.0 - lon) * kl * 111.2);
      const float d = sqrtf(dx * dx + dy * dy);
      if (d < nearKm) { nearKm = d; nearIdx = i; }
    }
  }
  // Campo di riferimento: la destinazione GO TO (da NEAREST FIELDS, FIELDS o GO HOME) se attiva, altrimenti il piu' vicino
  if (gpsValid && navGotoActive && navGotoIdx >= 0 && navGotoIdx < AIRFIELD_COUNT) {
    const double kl = cos(lat * DEG_TO_RAD);
    const float dy = (float)((AIRFIELDS[navGotoIdx].lat5 / 100000.0 - lat) * 111.2);
    const float dx = (float)((AIRFIELDS[navGotoIdx].lon5 / 100000.0 - lon) * kl * 111.2);
    nearKm = sqrtf(dx * dx + dy * dy);
    nearIdx = navGotoIdx;
  } else if (gpsValid && homePositionValid) {
    // Senza GO TO si va verso casa: entro 5 km dalla home il campo di casa e' il riferimento
    // (autozoom, numeri pista, vista della pista) anche se un altro campo e' un po' piu' vicino.
    const double kl = cos(lat * DEG_TO_RAD);
    const float hy = (float)((homeLatitude - lat) * 111.2), hx = (float)((homeLongitude - lon) * kl * 111.2);
    const float homeKm = sqrtf(hx * hx + hy * hy);
    const int hIdx = frHomeFieldIdx();
    if (hIdx >= 0 && homeKm <= 5.0f) {
      const float dy = (float)((AIRFIELDS[hIdx].lat5 / 100000.0 - lat) * 111.2);
      const float dx = (float)((AIRFIELDS[hIdx].lon5 / 100000.0 - lon) * kl * 111.2);
      nearKm = sqrtf(dx * dx + dy * dy);
      nearIdx = hIdx;
    } else if (hIdx < 0 && homeKm < nearKm) {
      nearKm = homeKm;   // casa non nell'elenco: l'autozoom segue comunque la home
    }
  }
  frFocusIdx = nearIdx;
  // Pagina informazioni del campo (tocco al centro): resta finche' non si tocca
  if (frInfoOpen && gpsValid && nearIdx >= 0) {
    frDrawFieldInfo(nearIdx, lat, lon, gsKmh);
    navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
    return;
  }
  // Vista della pista entro 2 km (si chiude oltre 2.5 km); uno swipe la spegne fino all'uscita oltre 2.5 km
  // Si apre a 2 km dalla pista, da qualsiasi direzione; si chiude oltre 2.5 km
  const float rwyKm = (gpsValid && nearIdx >= 0) ? frRunwayDist(nearIdx, lat, lon) : 1e9f;
  frRwyDistKm = rwyKm;
  if (!gpsValid || nearIdx < 0 || rwyKm > 5.5f) { frRwyView = false; frRwySuppressed = false; }
  else if (!frRwySuppressed && (rwyKm <= 5.0f || frRwyView)) frRwyView = true;
  if (frRwyView && nearIdx >= 0) {
    frCount = 0;
    frDrawRunwayView(nearIdx, lat, lon, course);
    // Vento dal telefono sulla vista pista: freccia blu sul bordo (il verde sul fondo sabbia
    // si leggerebbe male) e, sopra la pista, direzione e intensita' in un riquadro bianco.
    // Campo appena cambiato e meteo nuovo non ancora arrivato: niente freccia e niente
    // pallino, perche' sarebbero il vento dell'altro campo sulla pista di questo. Al loro
    // posto una scritta piccola e grigia, nel quadrante in alto a destra che qui e' libero.
    if (metWindValid && metFresh() && !metAligned()) {
      const uint16_t grigio = lgfx::color565(106, 99, 80);
      frText("WIND", 288, 92, MC_DATUM, grigio, 0.34f);
      frText("UPDATING", 288, 110, MC_DATUM, grigio, 0.34f);
    }
    if (metWindValid && metFresh() && metAligned()) {
      // I gradi non si scrivono: li leggi dalla bussola disegnata intorno, guardando da dove
      // arriva la freccia. Nel pallino sulla coda ci stanno solo i nodi.
      const bool forte = metWindStrong();
      const uint16_t wc = forte ? lgfx::color565(190, 25, 25) : lgfx::color565(0, 70, 160);
      drawWindArrow(navigationSprite, course, wc, 142, 98);
      const float rel = ((float)metWindDir - course) * DEG_TO_RAD;
      const int cx = (int)lroundf(180.0f + 142.0f * sinf(rel));
      const int cy = (int)lroundf(180.0f - 142.0f * cosf(rel));
      if (metPulseOn()) {   // dato nuovo appena arrivato: anelli blu, non rossi (vedi sopra)
        const uint16_t pc = lgfx::color565(0, 70, 160);
        navigationSprite.drawCircle(cx, cy, 26, pc);
        navigationSprite.drawCircle(cx, cy, 33, pc);
      }
      navigationSprite.fillCircle(cx, cy, 19, TFT_WHITE);
      navigationSprite.drawCircle(cx, cy, 19, wc);
      navigationSprite.drawCircle(cx, cy, 18, wc);
      windDotX = cx; windDotY = cy; windDotShown = true;
      if (!forte || metBlinkOn()) {   // vento forte o a raffiche: il numero lampeggia in rosso
        char kb[8];
        snprintf(kb, sizeof(kb), "%d", (int)lroundf((float)metWindKt));
        frText(kb, cx, cy + 1, MC_DATUM, wc, 0.52f);
      }
    }
    navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
    return;
  }
  // Autozoom sul campo di riferimento (GO TO o il piu' vicino), come uno swipe automatico:
  // oltre 20 km scala 30, da 20 a 10 km scala 20, da 10 a 5 km scala 10, poi 8 / 4 / 2 km.
  // Uno swipe lo ferma; riparte quando si passa alla scala successiva.
  if (!gpsValid || nearIdx < 0) { frAutoKm = 0.0f; frAutoSuppressed = false; frAutoBand = 0.0f; }
  else {
    float want;
    // da 2.2 a 5 km scala 5, da 1.1 a 2.2 km scala 2, sotto 1.1 km (o a 2 km o meno dalla pista) scala 1.5
    if (nearKm <= 5.0f) {
      want = (frRwyDistKm <= 2.0f || nearKm <= 1.1f) ? 1.5f : (nearKm <= 2.2f ? 2.0f : 5.0f);
    } else want = nearKm <= 10.0f ? 10.0f : (nearKm <= 20.0f ? 20.0f : 30.0f);
    // isteresi: si allarga solo con il 10 % di margine, niente salti avanti e indietro
    if (frAutoBand > 0.0f && want > frAutoBand) {
      const float lim = frAutoBand == 1.5f ? 1.1f : (frAutoBand == 2.0f ? 2.2f : frAutoBand);
      if (nearKm <= lim * 1.1f || (frAutoBand == 1.5f && frRwyDistKm <= 2.2f)) want = frAutoBand;
    }
    if (want != frAutoBand) {
      const bool wasOn = frAutoKm > 0.0f;
      frAutoBand = want;
      frAutoSuppressed = false;   // nuova scala: l'autozoom riparte
      if (wasOn && want != frAutoKm) playTone(1200, 50);   // come uno swipe
    }
    frAutoKm = frAutoSuppressed ? 0.0f : want;
  }
  const float viewKm = frAutoKm > 0.0f ? frAutoKm : (float)frRangeKm;
  FR_RANGE_KM = viewKm;
  FR_PX_PER_KM = 165.0f / viewKm;
  char title[24];
  if (frAutoKm > 0.0f) {
    if (fabsf(viewKm - roundf(viewKm)) < 0.01f) snprintf(title, sizeof(title), "FIELDS %d km AUTO", (int)lroundf(viewKm));
    else snprintf(title, sizeof(title), "FIELDS %.1f km AUTO", viewKm);
  }
  else snprintf(title, sizeof(title), "FIELDS %d km", frRangeKm);
  frCount = 0;
  if (gpsValid) {
    // CTR: rosa chiaro, banda interna rosa piu' scura, bordo blu tratteggiato (dal file spazi aerei)
    aspDrawCtrOutlines(g, lat, lon, course, FR_RANGE_KM, FR_PX_PER_KM);
  }
  // anelli: 0.5 km fino a 2 km, 1 km fino a 5 km, 2 km fino a 8 km, poi 5 e 10 km
  const float ringStep = viewKm <= 2.0f ? 0.5f : (viewKm <= 5.0f ? 1.0f : (viewKm <= 8.0f ? 2.0f : (viewKm <= 10.0f ? 5.0f : 10.0f)));
  for (float r = ringStep; r <= viewKm + 0.01f; r += ringStep) {
    const int rp = (int)lroundf(r * FR_PX_PER_KM);
    g.drawCircle(180, 180, rp, FR_C_RING);
    char t[10];
    const bool last = fabsf(r - viewKm) < 0.01f;
    if (fabsf(r - roundf(r)) < 0.01f) snprintf(t, sizeof(t), last ? "%d km" : "%d", (int)lroundf(r));
    else snprintf(t, sizeof(t), last ? "%.1f km" : "%.1f", r);
    frText(t, 180 + (int)(rp * 0.72f), 180 + (int)(rp * 0.72f) - 2, ML_DATUM, FR_C_RINGT, 0.36f);
  }
  if (!gpsValid) {   // ne' fix ne' posizione dal telefono: non c'e' niente da disegnare
    frText(title, 180, 50, MC_DATUM, FR_C_INK, 0.5f);
    frText("NO GPS FIX", 180, 180, MC_DATUM, FR_C_N, 0.9f);
    navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
    return;
  }
  if (frPhone) frText("POSITION FROM PHONE", 180, 70, MC_DATUM, FR_C_RINGT, 0.34f);
  // Campi entro il raggio, ordinati per distanza
  const double kLon = cos(lat * DEG_TO_RAD);
  float fd[FR_MAX], fb[FR_MAX];
  int n = 0;
  for (int i = 0; i < AIRFIELD_COUNT; ++i) {
    const float dyKm = (float)((AIRFIELDS[i].lat5 / 100000.0 - lat) * 111.2);
    const float dxKm = (float)((AIRFIELDS[i].lon5 / 100000.0 - lon) * kLon * 111.2);
    if (fabsf(dyKm) > FR_RANGE_KM || fabsf(dxKm) > FR_RANGE_KM) continue;
    const float d = sqrtf(dxKm * dxKm + dyKm * dyKm);
    if (d > FR_RANGE_KM) continue;
    int k;
    if (n < FR_MAX) k = n++;
    else { if (d >= fd[FR_MAX - 1]) continue; k = FR_MAX - 1; }   // elenco pieno: sostituisce il piu' lontano
    while (k > 0 && d < fd[k - 1]) { fd[k] = fd[k - 1]; fb[k] = fb[k - 1]; frIdx[k] = frIdx[k - 1]; --k; }
    fd[k] = d;
    float b = atan2f(dxKm, dyKm) / DEG_TO_RAD; if (b < 0) b += 360.0f;
    fb[k] = b; frIdx[k] = i;
  }
  frCount = n;
  // GO TO attivo: linea rossa dall'aereo al campo di destinazione (fino al bordo se e' fuori scala)
  if (navGotoActive && navGotoIdx >= 0 && navGotoIdx < AIRFIELD_COUNT) {
    const float dyKm = (float)((AIRFIELDS[navGotoIdx].lat5 / 100000.0 - lat) * 111.2);
    const float dxKm = (float)((AIRFIELDS[navGotoIdx].lon5 / 100000.0 - lon) * kLon * 111.2);
    const float dKm = sqrtf(dxKm * dxKm + dyKm * dyKm);
    if (dKm > 0.05f) {
      const float rel = (atan2f(dxKm, dyKm) / DEG_TO_RAD - course) * DEG_TO_RAD;
      const float px = min(dKm * FR_PX_PER_KM, 170.0f);
      const float ex = 180.0f + px * sinf(rel), ey = 180.0f - px * cosf(rel);
      g.drawWideLine(180.0f, 180.0f, ex, ey, 3.6f, FR_C_N);   // spessa il doppio
    }
  }
  // Simboli dei campi: cerchio blu con il trattino della pista (doppio cerchio se la pista e' >= 1200 m);
  // i 3 piu' vicini con un pallino verde, giallo, arancione; GO TO con un anello arancione
  const uint16_t rankC[3] = { FR_C_GREEN, FR_C_YEL, FR_C_ORG };
  for (int k = n - 1; k >= 0; --k) {
    const AirfieldRec &f = AIRFIELDS[frIdx[k]];
    int x, y; frProject(fd[k], fb[k], course, x, y);
    frX[k] = x; frY[k] = y;
    const bool big = f.rwLenM >= 1200;
    const int r = big ? 12 : 9;   // simboli piu' grandi del 30 %
    const float a = (airfieldRwyDeg(frIdx[k]) - course - 90.0f) * DEG_TO_RAD;
    // pista in scala se piu' lunga del simbolo (autozoom vicino al campo)
    const bool isFocus = frIdx[k] == frFocusIdx && fd[k] <= 5.0f;
    const float halfScale = (isFocus && f.rwLenM > 0) ? f.rwLenM / 2000.0f * FR_PX_PER_KM : 0.0f;
    if (halfScale > r + 10) {
      g.drawWideLine(x - halfScale * cosf(a), y - halfScale * sinf(a), x + halfScale * cosf(a), y + halfScale * sinf(a), 5.2f, FR_C_BLUE);
      for (float t = -halfScale + 6; t < halfScale - 6; t += 11.0f) {   // linea centrale bianca tratteggiata
        const float t2 = min(t + 6.0f, halfScale - 6);
        g.drawWideLine(x + t * cosf(a), y + t * sinf(a), x + t2 * cosf(a), y + t2 * sinf(a), 0.8f, TFT_WHITE);
      }
    } else {
      g.drawCircle(x, y, r, FR_C_BLUE); g.drawCircle(x, y, r - 1, FR_C_BLUE); g.drawCircle(x, y, r - 2, FR_C_BLUE);
      if (big) { g.drawCircle(x, y, r + 4, FR_C_BLUE); g.drawCircle(x, y, r + 5, FR_C_BLUE); }
      const float L = isFocus ? r + 13.0f : r + 5.0f;   // vicino: trattino piu' lungo
      g.drawWideLine(x - L * cosf(a), y - L * sinf(a), x + L * cosf(a), y + L * sinf(a), isFocus ? 2.6f : 2.0f, FR_C_BLUE);
    }
    if (k < 3) g.fillCircle(x, y, 4, rankC[k]);
    if (navGotoActive && navGotoIdx == frIdx[k]) { const int rr = r + (big ? 8 : 6); g.drawCircle(x, y, rr, FR_C_ORG); g.drawCircle(x, y, rr + 1, FR_C_ORG); g.drawCircle(x, y, rr + 2, FR_C_ORG); }
  }
  // numeri della pista (testate) per il campo piu' vicino entro 5 km
  int16_t rwyBox[2][4]; int nRwyBox = 0;
  float rwyAxis = 0.0f;
  int kf = -1;
  for (int k = 0; k < n; ++k) if (frIdx[k] == frFocusIdx) { kf = k; break; }
  if (kf >= 0 && fd[kf] <= 5.0f && AIRFIELDS[frIdx[kf]].rwy > 0) {
    const AirfieldRec &f = AIRFIELDS[frIdx[kf]];
    const int r = f.rwLenM >= 1200 ? 9 : 7;
    rwyAxis = (airfieldRwyDeg(frIdx[kf]) - course - 90.0f) * DEG_TO_RAD;
    const float half = max((float)(r + 10), f.rwLenM > 0 ? f.rwLenM / 2000.0f * FR_PX_PER_KM : 0.0f);
    const float L = half + 16.0f;
    const int d1 = f.rwy, d2 = (f.rwy + 18 - 1) % 36 + 1;
    for (int e = 0; e < 2; ++e) {
      const float sg = e == 0 ? -1.0f : 1.0f;   // testata d1 dalla parte opposta alla direzione della pista
      const int tx = frX[kf] + (int)lroundf(sg * L * cosf(rwyAxis)), ty = frY[kf] + (int)lroundf(sg * L * sinf(rwyAxis));
      char t[4]; snprintf(t, sizeof(t), "%02d", e == 0 ? d1 : d2);
      g.fillRoundRect(tx - 14, ty - 11, 28, 22, 4, TFT_WHITE);
      g.drawRoundRect(tx - 14, ty - 11, 28, 22, 4, FR_C_BLUE);
      frText(t, tx, ty + 1, MC_DATUM, TFT_BLACK, 0.48f);
      rwyBox[nRwyBox][0] = tx - 15; rwyBox[nRwyBox][1] = ty - 12; rwyBox[nRwyBox][2] = tx + 15; rwyBox[nRwyBox][3] = ty + 12; ++nRwyBox;
    }
  }
  // home
  int homeIconX = -1000, homeIconY = -1000;
  if (homePositionValid) {
    const float dyKm = (float)((homeLatitude - lat) * 111.2), dxKm = (float)((homeLongitude - lon) * kLon * 111.2);
    const float d = sqrtf(dxKm * dxKm + dyKm * dyKm);
    if (d <= FR_RANGE_KM) {
      float b = atan2f(dxKm, dyKm) / DEG_TO_RAD; if (b < 0) b += 360.0f;
      int x, y; frProject(d, b, course, x, y);
      int hx = x - 14, hy = y - 14;
      if (nRwyBox > 0 && kf >= 0 && abs(x - frX[kf]) < 30 && abs(y - frY[kf]) < 30) {   // home = campo con i numeri: di lato alla pista
        hx = frX[kf] + (int)lroundf(24.0f * cosf(rwyAxis + 1.5708f));
        hy = frY[kf] + (int)lroundf(24.0f * sinf(rwyAxis + 1.5708f));
      }
      g.fillTriangle(hx - 8, hy - 1, hx + 8, hy - 1, hx, hy - 9, FR_C_GREEN);
      g.fillRect(hx - 5, hy - 1, 11, 8, FR_C_GREEN);
      homeIconX = hx; homeIconY = hy;
    }
  }
  // nomi in maiuscolo (prima i piu' vicini), senza sovrapposizioni
  int16_t lab[FR_MAX + 10][4]; int nLab = 0;
  auto labelFree = [&](int x0, int y0, int x1, int y1) {
    if (y0 < 62 || y1 > 290) return false;
    const int cx = (x0 + x1) / 2 - 180, cy = (y0 + y1) / 2 - 180;
    if (cx * cx + cy * cy > 150 * 150 || x0 < 22 || x1 > 338) return false;
    for (int j = 0; j < nLab; ++j)
      if (x0 < lab[j][2] && x1 > lab[j][0] && y0 < lab[j][3] && y1 > lab[j][1]) return false;
    return true;
  };
  lab[nLab][0] = 150; lab[nLab][1] = 156; lab[nLab][2] = 210; lab[nLab][3] = 202; ++nLab;   // aereo
  for (int e = 0; e < nRwyBox; ++e) { for (int q = 0; q < 4; ++q) lab[nLab][q] = rwyBox[e][q]; ++nLab; }
  // punti cardinali: N rossa, E S W nere, con alone color fondo; sopra il riquadro in basso se ci finirebbero sopra
  int cardX[4], cardY[4];
  for (int c = 0; c < 4; ++c) {
    const float an = (c * 90.0f - course - 90.0f) * DEG_TO_RAD;
    cardX[c] = 180 + (int)lroundf(158.0f * cosf(an));
    cardY[c] = 180 + (int)lroundf(158.0f * sinf(an));
    if (cardX[c] > 80 && cardX[c] < 280 && cardY[c] > 286) cardY[c] = 284;
    lab[nLab][0] = cardX[c] - 14; lab[nLab][1] = cardY[c] - 15; lab[nLab][2] = cardX[c] + 14; lab[nLab][3] = cardY[c] + 15; ++nLab;   // lettere +20 % con alone
  }
  if (homeIconX > -1000) {   // la casetta non va coperta dai nomi
    lab[nLab][0] = homeIconX - 9; lab[nLab][1] = homeIconY - 10; lab[nLab][2] = homeIconX + 9; lab[nLab][3] = homeIconY + 8; ++nLab;
  }
  // il nome del campo di casa ha la precedenza, vicino alla casetta
  const int homeIdxF = frHomeFieldIdx();
  int homeK = -1;
  for (int k = 0; k < n; ++k) if (frIdx[k] == homeIdxF) { homeK = k; break; }
  if (homeIconX > -1000 && homeK < 0) {   // casa non nell'elenco dei campi: scritta HOME accanto alla casetta
    const int ox[4] = { 12, -12 - 48, 12, -12 - 48 }, oy[4] = { -12, -12, -34, 4 };
    for (int o = 0; o < 4; ++o) {
      const int x0 = homeIconX + ox[o], y0 = homeIconY + oy[o];
      if (!labelFree(x0, y0, x0 + 48, y0 + 23)) continue;
      lab[nLab][0] = x0; lab[nLab][1] = y0; lab[nLab][2] = x0 + 48; lab[nLab][3] = y0 + 23; ++nLab;
      frText("HOME", x0, y0 + 11, ML_DATUM, FR_C_GREEN, 0.6f);
      break;
    }
  }
  for (int kk = -1; kk < n; ++kk) {
    const int k = kk < 0 ? homeK : (kk == homeK ? -1 : kk);
    if (k < 0) continue;
    const AirfieldRec &f = AIRFIELDS[frIdx[k]];
    // nomi grandi (+70 % rispetto all'inizio), senza distanza (resta nel riquadro in basso);
    // con due parole su due righe: "MILANO" sopra, "LINATE" sotto (separatori: spazio, -, _, /)
    char nm[20], l1[14], l2[14];
    int j = 0;
    for (; f.name[j] && j < 18; ++j) nm[j] = (char)toupper((unsigned char)f.name[j]);
    nm[j] = '\0';
    char *sp = strpbrk(nm, " -_/");
    bool two = false;
    if (sp != nullptr && sp > nm && sp[1] != '\0') {
      *sp = '\0'; two = true;
      snprintf(l1, sizeof(l1), "%.10s", nm);
      const char *rest = sp + 1;
      while (*rest == ' ' || *rest == '-') ++rest;
      snprintf(l2, sizeof(l2), "%.10s", rest);
      if (l2[0] == '\0') two = false;
    }
    if (!two) snprintf(l1, sizeof(l1), "%.11s", nm);
    const char *a1 = l1;
    const int CW = 12, LH = 23;                     // larghezza carattere e altezza riga (scala 0.65)
    const int w = (int)max(strlen(a1), two ? strlen(l2) : (size_t)0) * CW;
    const int h = two ? 2 * LH : LH;
    const int ox[6] = { 15, 15, -15 - w, -15 - w, 17, -17 - w }, oy[6] = { 6, -6 - h, 6, -6 - h, -h / 2, -h / 2 };
    for (int o = 0; o < 6; ++o) {   // sotto o sopra a destra, poi a sinistra, poi di fianco
      const int x0 = frX[k] + ox[o], y0 = frY[k] + oy[o];
      if (!labelFree(x0, y0, x0 + w, y0 + h)) continue;
      lab[nLab][0] = x0; lab[nLab][1] = y0; lab[nLab][2] = x0 + w; lab[nLab][3] = y0 + h;
      if (nLab < FR_MAX + 9) ++nLab;
      frText(a1, x0, y0 + LH / 2, ML_DATUM, FR_C_INK, 0.65f);
      if (two) frText(l2, x0, y0 + LH + LH / 2, ML_DATUM, FR_C_INK, 0.65f);
      break;
    }
  }
  // aereo nero, muso in alto: la stessa sagoma della bussola (AirplaneBitmap.h), ridotta a circa 48 px
  {
    static LGFX_Sprite frPlane;
    static bool frPlaneReady = false, frPlaneFailed = false;
    const uint16_t KEY = 0xF81F;   // colore trasparente (magenta)
    if (!frPlaneReady && !frPlaneFailed) {
      frPlane.setPsram(psramFound());
      frPlane.setColorDepth(16);
      if (frPlane.createSprite(120, 116) != nullptr) {
        frPlane.fillSprite(KEY);
        drawAirplaneSprite(frPlane, 60, 58, TFT_BLACK);   // sagoma piena 115 x 112 px
        frPlaneReady = true;
      } else {
        frPlaneFailed = true;
      }
    }
    if (frPlaneReady) frPlane.pushRotateZoomWithAA(&navigationSprite, 180.0f, 180.0f, 0.0f, 0.42f, 0.42f, KEY);
    else frDrawPlane(g, TFT_BLACK, FR_C_BG);   // memoria insufficiente: aereo semplice
  }
  // punti cardinali (ruotati con la rotta): +20 % (scala 0.72), alone bianco di 2 px con bordino
  // grigio-azzurro fuori, cosi' staccano sia dal fondo crema sia dal rosa delle CTR.
  // Carattere caricato una volta sola per tutte le lettere (l'alone ridisegna la lettera piu' volte).
  {
    static const char *const CARD[4] = { "N", "E", "S", "W" };
    const float sc = 0.72f;
    bool loaded = vlwSet(navigationSprite, UiFont32);
    if (loaded) navigationSprite.setTextSize(sc, sc);
    else { navigationSprite.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); navigationSprite.setTextSize(sc * 1.5f, sc * 1.5f); }
    navigationSprite.setTextDatum(MC_DATUM);
    for (int c = 0; c < 4; ++c) {
      navigationSprite.setTextColor(FR_C_RING);   // bordino esterno (distanza ~3 px)
      for (int ox = -3; ox <= 3; ++ox) for (int oy = -3; oy <= 3; ++oy) {
        const int d2 = ox * ox + oy * oy;
        if (d2 >= 6 && d2 <= 10) navigationSprite.drawString(CARD[c], cardX[c] + ox, cardY[c] + oy);
      }
      navigationSprite.setTextColor(TFT_WHITE);   // alone bianco (fino a 2 px)
      for (int ox = -2; ox <= 2; ++ox) for (int oy = -2; oy <= 2; ++oy) {
        const int d2 = ox * ox + oy * oy;
        if (d2 > 0 && d2 <= 5) navigationSprite.drawString(CARD[c], cardX[c] + ox, cardY[c] + oy);
      }
      navigationSprite.setTextColor(c == 0 ? FR_C_N : TFT_BLACK);
      navigationSprite.drawString(CARD[c], cardX[c], cardY[c]);
    }
    if (loaded) vlwRelease(navigationSprite);
  }
  frText(title, 180, 50, MC_DATUM, FR_C_INK, 0.5f);
  // GO TO attivo: riquadro arancione con la sola distanza che manca (carattere grande) e pulsante HOME
  if (navGotoActive && navGotoIdx >= 0) {
    const double kl = cos(lat * DEG_TO_RAD);
    const float dy = (float)((AIRFIELDS[navGotoIdx].lat5 / 100000.0 - lat) * 111.2);
    const float dx = (float)((AIRFIELDS[navGotoIdx].lon5 / 100000.0 - lon) * kl * 111.2);
    const float dKm = sqrtf(dx * dx + dy * dy);
    char line[20];
    if (dKm < 100.0f) snprintf(line, sizeof(line), "%.1f km", dKm); else snprintf(line, sizeof(line), "%d km", (int)lroundf(dKm));
    g.fillRoundRect(100, 252, 160, 42, 9, TFT_WHITE);
    g.drawRoundRect(100, 252, 160, 42, 9, FR_C_ORG); g.drawRoundRect(101, 253, 158, 40, 8, FR_C_ORG);
    frText(line, 180, 274, MC_DATUM, FR_C_ORG, 0.84f);   // il doppio di prima
    g.fillRoundRect(FR_HOME_X, FR_HOME_Y, FR_HOME_W, FR_HOME_H, 8, TFT_WHITE);
    const uint16_t gold = lgfx::color565(199, 154, 0);
    g.drawRoundRect(FR_HOME_X, FR_HOME_Y, FR_HOME_W, FR_HOME_H, 8, gold); g.drawRoundRect(FR_HOME_X + 1, FR_HOME_Y + 1, FR_HOME_W - 2, FR_HOME_H - 2, 7, gold);
    frTextFit("HOME", 180, FR_HOME_Y + FR_HOME_H / 2 + 1, FR_C_INK, 0.88f, FR_HOME_W - 10);   // grande, quasi a toccare il riquadro
    if (metWindValid && metFresh() && metAligned() && gpsValid) {
      const uint16_t gwc = metWindStrong() ? lgfx::color565(230, 60, 60) : lgfx::color565(61, 209, 122);
      drawWindArrow(navigationSprite, course, gwc, 140, 100);
      const float relG = ((float)metWindDir - course) * DEG_TO_RAD;
      const int gcx = (int)lroundf(180.0f + 140.0f * sinf(relG));
      const int gcy = (int)lroundf(180.0f - 140.0f * cosf(relG));
      if (metPulseOn()) {   // dato nuovo appena arrivato: anelli blu
        const uint16_t pc = lgfx::color565(0, 70, 160);
        navigationSprite.drawCircle(gcx, gcy, 26, pc);
        navigationSprite.drawCircle(gcx, gcy, 33, pc);
      }
      navigationSprite.fillCircle(gcx, gcy, 19, TFT_WHITE);
      navigationSprite.drawCircle(gcx, gcy, 19, gwc);
      navigationSprite.drawCircle(gcx, gcy, 18, gwc);
      if (!metWindStrong() || metBlinkOn()) {
        char gkb[8];
        snprintf(gkb, sizeof(gkb), "%d", (int)lroundf((float)metWindKt));
        frText(gkb, gcx, gcy + 1, MC_DATUM, gwc, 0.52f);
      }
      windDotX = gcx; windDotY = gcy; windDotShown = true;
    }
    navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
    return;
  }
  // riquadro bianco in basso: campo piu' vicino (distanza e rotta vera) e numero di campi
  g.fillRoundRect(92, 296, 176, 50, 8, TFT_WHITE);
  g.drawRoundRect(92, 296, 176, 50, 8, FR_C_RING);
  char line[40];
  if (n > 0) {
    char nm[14]; int j = 0;
    for (; AIRFIELDS[frIdx[0]].name[j] && j < 10; ++j) nm[j] = (char)toupper((unsigned char)AIRFIELDS[frIdx[0]].name[j]);
    nm[j] = '\0';
    snprintf(line, sizeof(line), "%s %.1f km %03d", nm, fd[0], (int)lroundf(fb[0]) % 360);
    const int w = frText(line, 180, 312, MC_DATUM, FR_C_GREEN, 0.42f);
    g.drawCircle(180 + w / 2 + 4, 306, 2, FR_C_GREEN);   // simbolo dei gradi
    snprintf(line, sizeof(line), "%d fields within %d km", n, (int)FR_RANGE_KM);
  } else {
    snprintf(line, sizeof(line), "No fields within %d km", (int)FR_RANGE_KM);
  }
  frText(line, 180, 333, MC_DATUM, FR_C_INK, 0.36f);
  // Vento dal telefono: freccia sul bordo e pallino con i nodi sulla coda, come in GO TO HOME.
  if (metWindValid && metFresh() && metAligned() && gpsValid) {
    const uint16_t fwc = metWindStrong() ? lgfx::color565(230, 60, 60) : lgfx::color565(61, 209, 122);
    drawWindArrow(navigationSprite, course, fwc, 140, 100);
    const float relF = ((float)metWindDir - course) * DEG_TO_RAD;
    const int fcx = (int)lroundf(180.0f + 140.0f * sinf(relF));
    const int fcy = (int)lroundf(180.0f - 140.0f * cosf(relF));
    if (metPulseOn()) {   // dato nuovo appena arrivato: anelli blu
      const uint16_t pc = lgfx::color565(0, 70, 160);
      navigationSprite.drawCircle(fcx, fcy, 26, pc);
      navigationSprite.drawCircle(fcx, fcy, 33, pc);
    }
    navigationSprite.fillCircle(fcx, fcy, 19, TFT_WHITE);
    navigationSprite.drawCircle(fcx, fcy, 19, fwc);
    navigationSprite.drawCircle(fcx, fcy, 18, fwc);
    if (!metWindStrong() || metBlinkOn()) {
      char fkb[8];
      snprintf(fkb, sizeof(fkb), "%d", (int)lroundf((float)metWindKt));
      frText(fkb, fcx, fcy + 1, MC_DATUM, fwc, 0.52f);
    }
    windDotX = fcx; windDotY = fcy; windDotShown = true;
  }
  navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

// Swipe verticale: dir -1 = swipe in su (zoom piu' vicino), +1 = swipe in giu' (piu' lontano)
void fieldsRadarZoom(int dir) {
  if (frRwyView) {   // vista pista: lo swipe torna alla vista dall'alto fino all'uscita oltre 2.5 km
    frRwyView = false;
    frRwySuppressed = true;
    playTone(1200, 50);
    updateFieldsRadarScreen(true);
    return;
  }
  if (frAutoKm > 0.0f) {   // autozoom attivo: lo swipe lo spegne e torna allo zoom scelto
    frAutoKm = 0.0f;
    frAutoSuppressed = true;
    playTone(1200, 50);
    updateFieldsRadarScreen(true);
    return;
  }
  const int next = frRangeKm + dir * 10;
  if (next < 10 || next > 30) { playTone(600, 60); return; }   // gia' al minimo o al massimo
  frApplyRange(next);
  if (preferences.begin("pages", false)) { preferences.putUChar("frzoom", (uint8_t)frRangeKm); preferences.end(); }
  playTone(1200, 50);
  updateFieldsRadarScreen(true);
}

// Tocco: il campo piu' vicino al dito (entro 24 px) diventa la destinazione GO TO
void fieldsRadarTap(int tx, int ty) {
  if (frInfoOpen) {   // pagina informazioni: un tocco chiude
    frInfoOpen = false;
    playTone(1200, 50);
    updateFieldsRadarScreen(true);
    return;
  }
  if (navGotoActive && !frRwyView && tx >= FR_HOME_X - 6 && tx <= FR_HOME_X + FR_HOME_W + 6 &&
      ty >= FR_HOME_Y - 6 && ty <= FR_HOME_Y + FR_HOME_H + 6) {   // HOME: di nuovo verso casa (anche sull'altro strumento)
    navCancelGoto();
    bleQueuePrefs();
    playTone(1500, 80);
    updateFieldsRadarScreen(true);
    return;
  }
  const int infoR = frRwyView ? 90 : 36;   // vista pista: basta toccare la pista
  if ((tx - 180) * (tx - 180) + (ty - 180) * (ty - 180) <= infoR * infoR && frFocusIdx >= 0) {   // centro: informazioni del campo
    frInfoOpen = true;
    playTone(1200, 50);
    updateFieldsRadarScreen(true);
    return;
  }
  int best = -1, bestD2 = 24 * 24;
  for (int k = 0; k < frCount; ++k) {
    const int dx = tx - frX[k], dy = ty - frY[k], d2 = dx * dx + dy * dy;
    if (d2 < bestD2) { bestD2 = d2; best = k; }
  }
  if (best < 0) return;
  navSetGoto(frIdx[best]);
  bleQueuePrefs();
  playTone(1500, 80);
  currentScreen = GO_TO_HOME_SCREEN;
  for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == GO_TO_HOME_SCREEN) { currentScreenIndex = i; break; }
  saveScreenPreference();
  drawGoToHomeScreen();
}

// (pagina COORDS della bussola eliminata: al suo posto nel riquadro c'e' la scritta A.V.E.A)

// ========== HOME COORDS: inserimento manuale delle coordinate home (rulli a scorrimento) ==========
// Si apre da HOME FIELD toccando COORDS: prima la pagina COORD FORMAT (XCSOAR, GARMIN, SKY DEMON,
// GOOGLE MAPS), poi tre rulli adattati al formato scelto, con trascinamento
// su/giu' e inerzia (scorrendo veloce il rullo accelera). In alto: N/S (o E/W) per l'emisfero e
// LON/LAT per passare all'altra coordinata. SAVE salva (NVS + slave via BLE) e torna a HOME FIELD.
// Swipe a destra o a sinistra: esce senza salvare. Pagina sempre a colori, swipe verticale senza cambio tema.

// Applica una home (da SET HOME, da COORDS o ricevuta via BLE): variabili, NVS e ridisegno.
void applyHomeLocal(bool valid, double lat, double lon, bool redraw) {
  homePositionValid = valid;
  homeLatitude = valid ? lat : 0.0;
  homeLongitude = valid ? lon : 0.0;
  demoHomeAutoSet = false;          // ora e' una home vera: la fine della demo non deve cancellarla
  lubberLegValid = false;
  goToHomeNeedsRedraw = true;
  preferences.begin("altimeter", false);
  preferences.putBool("home_valid", valid);
  if (valid) {
    preferences.putDouble("home_lat", homeLatitude);
    preferences.putDouble("home_lon", homeLongitude);
  } else {
    preferences.remove("home_lat");
    preferences.remove("home_lon");
  }
  preferences.end();
  if (!redraw) return;
  if (currentScreen == HOME_FIELD_SCREEN) drawHomeFieldScreen();
  else if (currentScreen == GO_TO_HOME_SCREEN) drawGoToHomeScreen();
}

// ---- Formati delle coordinate (pagina FORMAT, prima dei rulli) ----
// 0 XCSOAR      gradi, primi, secondi          N45°27'30"   (formato predefinito di XCSoar)
// 1 GARMIN      gradi, primi decimali          N45°27.500'  (hddd°mm.mmm' dei GPS Garmin)
// 2 SKY DEMON   gradi, primi, secondi compatti N452730      (formato standard dell'inserimento manuale)
// 3 GOOGLE MAPS gradi decimali con segno       45.458333    (6 decimali)
// Esempi mostrati sotto il nome ('*' = simbolo dei gradi, disegnato a mano)
static const char *const HC_FMT_SAMPLES[HC_FMT_COUNT] = {
  "N45*27'30\"  E009*11'20\"", "N45*27.500'  E009*11.333'", "N452730  E0091120", "45.458333, 9.188889"
};

static const int HC_ROW = 38;               // passo verticale delle righe dei rulli (px)
static const int HC_CY = 180;               // riga centrale (valore selezionato)
static const int HC_AREA_Y0 = 108, HC_AREA_Y1 = 252;  // zona visibile dei rulli
static const int HC_TOUCH_Y0 = 84, HC_TOUCH_Y1 = 270; // zona di trascinamento
static const int HC_BAND_Y0 = 157, HC_BAND_Y1 = 203;  // fascia di selezione
static const int HC_COL_DEG_RX = 128;       // gradi allineati a destra
static const int HC_COL1_CX = 180;          // secondo rullo centrato
static const int HC_COL2_CX = 262;          // terzo rullo centrato (XCSOAR, SKY DEMON, GOOGLE)
static const int HC_COL2_LX = 228;          // terzo rullo allineato a sinistra dopo il punto (GARMIN)
static double hcDeg[2] = {0.0, 0.0};        // valori di partenza in gradi (LAT, LON)
static float hcPos[2][3];                   // [LAT/LON][rullo]: posizione continua del rullo
static bool hcNeg[2];                       // true = S / W (oppure segno meno in GOOGLE MAPS)
static int hcField = 0;                     // 0 = LAT, 1 = LON
static int hcDragCol = -1;                  // rullo trascinato (-1 nessuno)
static bool hcDragDecided = false;
static unsigned long hcTouchId = 0;
static int hcLastY = 0;
static unsigned long hcLastMs = 0;
static float hcVel = 0.0f;                  // passi/ms durante il trascinamento
static int hcFlingCol = -1;                 // rullo che sta girando per inerzia
static float hcFlingVel = 0.0f;
static unsigned long hcFlingLastMs = 0;
static unsigned long hcLastDrawMs = 0;

// Valori per rullo: secondo e terzo dipendono dal formato.
static int hcRange(int field, int col) {
  if (col == 0) return field == 0 ? 91 : 181;
  if (hcFormat == HC_FMT_GOOGLE) return 1000;                       // .458 333
  if (col == 1) return 60;                                          // primi
  return hcFormat == HC_FMT_GARMIN ? 1000 : 60;                     // millesimi di primo / secondi
}
static long hcUnitsPerDeg() { return (long)hcRange(0, 1) * (long)hcRange(0, 2); }   // 3600, 60000, 1000000
static int hcWrap(int v, int n) { return ((v % n) + n) % n; }
static int hcValue(int field, int col) { return hcWrap((int)lroundf(hcPos[field][col]), hcRange(field, col)); }
static void hcKeepBounded(int field, int col) {
  const float n = (float)hcRange(field, col);
  hcPos[field][col] = fmodf(hcPos[field][col], n);
  if (hcPos[field][col] < 0.0f) hcPos[field][col] += n;
}
// Arrotonda il rullo al valore piu' vicino; a 90° (lat) o 180° (lon) gli altri rulli tornano a 0.
static void hcSnap(int field, int col) {
  hcPos[field][col] = (float)hcValue(field, col);
  if (hcValue(field, 0) == hcRange(field, 0) - 1) { hcPos[field][1] = 0.0f; hcPos[field][2] = 0.0f; }
}
static void hcSetFromDegrees(int field, double v) {
  hcNeg[field] = v < 0.0;
  const long unit = hcUnitsPerDeg();
  const long r2 = hcRange(field, 2);
  long t = (long)llround(fabs(v) * (double)unit);
  const long maxT = (long)(hcRange(field, 0) - 1) * unit;
  if (t > maxT) t = maxT;
  hcPos[field][0] = (float)(t / unit);
  hcPos[field][1] = (float)((t % unit) / r2);
  hcPos[field][2] = (float)(t % r2);
}
static double hcDegrees(int field) {
  const double v = hcValue(field, 0) +
                   ((double)hcValue(field, 1) * hcRange(field, 2) + hcValue(field, 2)) / (double)hcUnitsPerDeg();
  return hcNeg[field] ? -v : v;
}

static void hcText(lgfx::LGFXBase &g, const char *txt, int x, int y, float scale, uint16_t fg,
                   decltype(MC_DATUM) datum) {
  const bool loaded = vlwSet(g, UiFont32);
  if (loaded) g.setTextSize(scale, scale);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(scale * 1.4f, scale * 1.4f); }
  g.setTextDatum(datum);
  g.setTextColor(fg);   // sfondo trasparente: le righe possono passare sopra la fascia di selezione
  g.drawString(txt, x, y);
  if (loaded) vlwRelease(g);
}
// Testo centrato in cx dove '*' diventa il simbolo dei gradi (cerchietto disegnato a mano).
static void hcTextDeg(lgfx::LGFXBase &g, const char *txt, int cx, int y, float scale, uint16_t fg) {
  const bool loaded = vlwSet(g, UiFont32);
  if (loaded) g.setTextSize(scale, scale);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b); g.setTextSize(scale * 1.6f, scale * 1.6f); }
  g.setTextDatum(ML_DATUM);
  g.setTextColor(fg);
  const int degW = 8;
  char piece[40];
  int total = 0;
  for (const char *p = txt; ; ) {   // larghezza totale
    const char *star = strchr(p, '*');
    const size_t n = star ? (size_t)(star - p) : strlen(p);
    memcpy(piece, p, min(n, sizeof(piece) - 1)); piece[min(n, sizeof(piece) - 1)] = '\0';
    total += g.textWidth(piece);
    if (!star) break;
    total += degW; p = star + 1;
  }
  const int fh = g.fontHeight();
  int x = cx - total / 2;
  for (const char *p = txt; ; ) {
    const char *star = strchr(p, '*');
    const size_t n = star ? (size_t)(star - p) : strlen(p);
    memcpy(piece, p, min(n, sizeof(piece) - 1)); piece[min(n, sizeof(piece) - 1)] = '\0';
    g.drawString(piece, x, y);
    x += g.textWidth(piece);
    if (!star) break;
    g.drawCircle(x + degW / 2, y - fh / 4, 2, fg);
    x += degW; p = star + 1;
  }
  if (loaded) vlwRelease(g);
}
// Coordinata nel testo dell'app scelta ('*' = gradi), per la riga di anteprima.
static void hcFormatCoord(int field, char *out, size_t outSize) {
  const char h = field == 0 ? (hcNeg[field] ? 'S' : 'N') : (hcNeg[field] ? 'W' : 'E');
  const int d = hcValue(field, 0), a = hcValue(field, 1), b = hcValue(field, 2);
  const char *dfmt = field == 0 ? "%02d" : "%03d";
  char ds[4]; snprintf(ds, sizeof(ds), dfmt, d);
  switch (hcFormat) {
    case HC_FMT_XCSOAR:   snprintf(out, outSize, "%c%s*%02d'%02d\"", h, ds, a, b); break;
    case HC_FMT_GARMIN:   snprintf(out, outSize, "%c%s*%02d.%03d'", h, ds, a, b); break;
    case HC_FMT_SKYDEMON: snprintf(out, outSize, "%c%s%02d%02d", h, ds, a, b); break;
    default:              snprintf(out, outSize, "%s%d.%03d%03d", hcNeg[field] ? "-" : "", d, a, b); break;
  }
}

// ---- Pagina FORMAT: scelta del formato prima dei rulli ----
static const int HC_FMT_BTN_X = 58, HC_FMT_BTN_W = 244, HC_FMT_BTN_H = 48, HC_FMT_BTN_Y0 = 76, HC_FMT_BTN_STEP = 54;

void drawHomeCoordsFormatScreen() {
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  const uint16_t gold = lgfx::color565(239, 159, 39);
  const uint16_t green = lgfx::color565(29, 158, 117);
  g.fillScreen(TFT_BLACK);
  hcText(g, "COORD FORMAT", 180, 46, 0.72f, TFT_CYAN, MC_DATUM);
  for (int i = 0; i < HC_FMT_COUNT; ++i) {
    const int y = HC_FMT_BTN_Y0 + i * HC_FMT_BTN_STEP;
    const bool last = (i == hcFormat);   // ultimo formato usato evidenziato
    const uint16_t c = last ? green : gold;
    g.drawRoundRect(HC_FMT_BTN_X, y, HC_FMT_BTN_W, HC_FMT_BTN_H, 8, c);
    g.drawRoundRect(HC_FMT_BTN_X + 1, y + 1, HC_FMT_BTN_W - 2, HC_FMT_BTN_H - 2, 7, c);
    hcText(g, HC_FMT_NAMES[i], 180, y + 15, 0.6f, c, MC_DATUM);
    hcTextDeg(g, HC_FMT_SAMPLES[i], 180, y + 35, 0.42f, lgfx::color565(170, 170, 170));
  }
  hcText(g, "Swipe to exit", 180, 316, 0.45f, lgfx::color565(170, 170, 170), MC_DATUM);
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

void drawHomeCoordsScreen() {
  hcLastDrawMs = millis();
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  const uint16_t gold = lgfx::color565(239, 159, 39);
  const uint16_t green = lgfx::color565(29, 158, 117);
  const uint16_t band = lgfx::color565(26, 26, 26);
  const uint16_t grey = lgfx::color565(170, 170, 170);
  g.fillScreen(TFT_BLACK);

  // Intestazione: emisfero o segno (tocco = inverte), titolo con formato, passaggio all'altra coordinata
  char hemi[2];
  if (hcFormat == HC_FMT_GOOGLE) hemi[0] = hcNeg[hcField] ? '-' : '+';
  else hemi[0] = hcField == 0 ? (hcNeg[0] ? 'S' : 'N') : (hcNeg[1] ? 'W' : 'E');
  hemi[1] = 0;
  g.drawRoundRect(72, 46, 56, 34, 8, gold);
  g.drawRoundRect(73, 47, 54, 32, 7, gold);
  hcText(g, hemi, 100, 63, 0.9f, gold, MC_DATUM);
  hcText(g, hcField == 0 ? "LAT" : "LON", 180, 57, 0.85f, TFT_CYAN, MC_DATUM);
  hcText(g, HC_FMT_NAMES[hcFormat], 180, 80, 0.36f, grey, MC_DATUM);
  g.drawRoundRect(222, 46, 70, 34, 8, gold);
  hcText(g, hcField == 0 ? "LON" : "LAT", 247, 63, 0.62f, gold, MC_DATUM);
  g.fillTriangle(274, 55, 274, 71, 284, 63, gold);

  // Fascia di selezione
  g.fillRoundRect(40, HC_BAND_Y0, 280, HC_BAND_Y1 - HC_BAND_Y0, 8, band);
  g.drawRoundRect(40, HC_BAND_Y0, 280, HC_BAND_Y1 - HC_BAND_Y0, 8, green);
  g.drawRoundRect(41, HC_BAND_Y0 + 1, 278, HC_BAND_Y1 - HC_BAND_Y0 - 2, 7, green);

  // Rulli: righe sopra e sotto in grigio
  g.setClipRect(0, HC_AREA_Y0, 360, HC_AREA_Y1 - HC_AREA_Y0);
  for (int col = 0; col < 3; ++col) {
    const int n = hcRange(hcField, col);
    const int base = (int)lroundf(hcPos[hcField][col]);
    const float frac = hcPos[hcField][col] - (float)base;
    for (int k = -2; k <= 2; ++k) {
      const int y = (int)lroundf(HC_CY + ((float)k - frac) * HC_ROW);
      const int dist = abs(y - HC_CY);
      const int v = hcWrap(base + k, n);
      char buf[6];
      if (col == 0) snprintf(buf, sizeof(buf), hcFormat == HC_FMT_GOOGLE ? "%d" : (hcField == 0 ? "%02d" : "%03d"), v);
      else snprintf(buf, sizeof(buf), n >= 1000 ? "%03d" : "%02d", v);
      const uint16_t c = dist < HC_ROW / 2 ? (uint16_t)TFT_WHITE
                       : (dist < HC_ROW * 3 / 2 ? lgfx::color565(136, 136, 136) : lgfx::color565(68, 68, 68));
      const float sc = dist < HC_ROW / 2 ? 1.05f : 0.8f;
      if (col == 0) hcText(g, buf, HC_COL_DEG_RX, y, sc, c, MR_DATUM);
      else if (col == 1) hcText(g, buf, HC_COL1_CX, y, sc, c, MC_DATUM);
      else if (hcFormat == HC_FMT_GARMIN) hcText(g, buf, HC_COL2_LX, y, sc, c, ML_DATUM);
      else hcText(g, buf, HC_COL2_CX, y, sc, c, MC_DATUM);
    }
  }
  g.clearClipRect();

  // Simboli fissi nella fascia, secondo il formato
  const uint16_t w = TFT_WHITE;
  if (hcFormat == HC_FMT_XCSOAR || hcFormat == HC_FMT_GARMIN) {
    g.drawCircle(HC_COL_DEG_RX + 8, HC_CY - 10, 4, w);          // °
    g.drawCircle(HC_COL_DEG_RX + 8, HC_CY - 10, 3, w);
    g.fillRect(HC_COL1_CX + 24, HC_CY - 15, 3, 9, w);           // '
  }
  if (hcFormat == HC_FMT_GARMIN) g.fillCircle(HC_COL2_LX - 6, HC_CY + 10, 3, w);   // .mmm
  if (hcFormat == HC_FMT_XCSOAR) {                               // "
    g.fillRect(HC_COL2_CX + 24, HC_CY - 15, 3, 9, w);
    g.fillRect(HC_COL2_CX + 29, HC_CY - 15, 3, 9, w);
  }
  if (hcFormat == HC_FMT_GOOGLE) g.fillCircle(HC_COL_DEG_RX + 9, HC_CY + 10, 3, w);  // punto decimale

  // Anteprima: tutte e due le coordinate come le scrive l'app scelta
  char latTxt[24], lonTxt[24], line[56];
  hcFormatCoord(0, latTxt, sizeof(latTxt));
  hcFormatCoord(1, lonTxt, sizeof(lonTxt));
  snprintf(line, sizeof(line), hcFormat == HC_FMT_GOOGLE ? "%s, %s" : "%s  %s", latTxt, lonTxt);
  hcTextDeg(g, line, 180, 266, 0.5f, gold);

  // SAVE
  g.drawRoundRect(130, 284, 100, 38, 8, gold);
  g.drawRoundRect(131, 285, 98, 36, 7, gold);
  hcText(g, "SAVE", 180, 303, 0.75f, gold, MC_DATUM);
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

// Apertura da HOME FIELD (tocco su COORDS): valori di partenza = home salvata, altrimenti GPS, altrimenti 0.
// Prima si sceglie il formato.
void openHomeCoordsScreen() {
  double lat = 0.0, lon = 0.0;
  if (homePositionValid) { lat = homeLatitude; lon = homeLongitude; }
  else if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    if (gpsDataValid) { lat = gps_latitude; lon = gps_longitude; }
    xSemaphoreGive(sensorDataMutex);
  }
  hcDeg[0] = lat;
  hcDeg[1] = lon;
  preferences.begin("altimeter", true);
  hcFormat = constrain(preferences.getInt("hc_fmt", HC_FMT_GARMIN), 0, HC_FMT_COUNT - 1);
  preferences.end();
  currentScreen = HOME_COORDS_FMT_SCREEN;
  drawHomeCoordsFormatScreen();
}

// Scelta del formato: i rulli partono dai valori di apertura convertiti nel formato scelto.
static void hcEnterWheels(int fmt) {
  hcFormat = fmt;
  preferences.begin("altimeter", false);
  preferences.putInt("hc_fmt", hcFormat);
  preferences.end();
  hcSetFromDegrees(0, hcDeg[0]);
  hcSetFromDegrees(1, hcDeg[1]);
  // Per non sbagliare: i rulli partono sempre da N ed E (+ e + in GOOGLE MAPS).
  // Il tasto in alto a sinistra resta per invertirli a mano se servisse.
  hcNeg[0] = false;
  hcNeg[1] = false;
  hcField = 0;
  hcDragCol = -1; hcDragDecided = false; hcTouchId = 0;
  hcFlingCol = -1; hcFlingVel = 0.0f;
  currentScreen = HOME_COORDS_SCREEN;
  drawHomeCoordsScreen();
}

// Rilascio sulla pagina FORMAT: tocco su un formato = rulli; swipe dx/sx = HOME FIELD.
void homeCoordsFormatTouchRelease(bool tapValid, int diffX, int diffY, int startX, int startY) {
  if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
    currentScreen = HOME_FIELD_SCREEN;
    drawHomeFieldScreen();
    playTone(1500, 80);
    return;
  }
  if (!tapValid || startX < HC_FMT_BTN_X - 15 || startX > HC_FMT_BTN_X + HC_FMT_BTN_W + 15) return;
  for (int i = 0; i < HC_FMT_COUNT; ++i) {
    const int y = HC_FMT_BTN_Y0 + i * HC_FMT_BTN_STEP;
    if (startY >= y - 3 && startY <= y + HC_FMT_BTN_H + 3) {
      playTone(1500, 80);
      hcEnterWheels(i);
      return;
    }
  }
}

static int hcColumnAt(int x) { return x < 145 ? 0 : (x < 215 ? 1 : 2); }

// Dito appoggiato: trascinamento verticale di un rullo (con accelerazione se veloce).
void homeCoordsTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != HOME_COORDS_SCREEN) return;
  const unsigned long now = millis();
  if (touchId != hcTouchId) {
    hcTouchId = touchId;
    hcDragCol = -1; hcDragDecided = false;
    hcLastY = startY; hcLastMs = now; hcVel = 0.0f;
    if (hcFlingCol >= 0) {   // il tocco ferma il rullo che gira
      hcSnap(hcField, hcFlingCol);
      hcFlingCol = -1; hcFlingVel = 0.0f;
      drawHomeCoordsScreen();
    }
    return;
  }
  if (!hcDragDecided) {
    const int dx = abs(x - startX), dy = abs(y - startY);
    if (dx < 8 && dy < 8) return;
    hcDragDecided = true;
    if (dy > dx && startY >= HC_TOUCH_Y0 && startY <= HC_TOUCH_Y1) hcDragCol = hcColumnAt(startX);
  }
  if (hcDragCol < 0) return;
  const int d = y - hcLastY;
  const unsigned long dt = max(1UL, now - hcLastMs);
  hcLastY = y; hcLastMs = now;
  if (d == 0) return;
  const float speed = fabsf((float)d) / (float)dt;   // px/ms
  const int n = hcRange(hcField, hcDragCol);
  float gain;
  if (n >= 1000) gain = constrain(1.0f + (speed - 0.25f) * 14.0f, 1.0f, 25.0f);   // 1000 valori: fino a x25
  else if (n > 100) gain = constrain(1.0f + (speed - 0.4f) * 3.0f, 1.0f, 4.0f);   // 0-180 gradi
  else gain = constrain(1.0f + (speed - 0.5f) * 1.5f, 1.0f, 2.5f);
  const float steps = -(float)d / (float)HC_ROW * gain;   // dito verso l'alto = valori piu' grandi
  hcPos[hcField][hcDragCol] += steps;
  hcKeepBounded(hcField, hcDragCol);
  hcVel = 0.6f * hcVel + 0.4f * (steps / (float)dt);
  if (now - hcLastDrawMs >= 30UL) drawHomeCoordsScreen();
}

// Rilascio del dito sui rulli. Gestisce tutto il tocco della pagina (anche lo swipe di uscita).
void homeCoordsTouchRelease(bool tapValid, int diffX, int diffY, int startX, int startY) {
  const unsigned long now = millis();
  if (hcDragCol >= 0) {
    const int col = hcDragCol;
    hcDragCol = -1; hcDragDecided = false; hcTouchId = 0;
    if (now - hcLastMs < 80UL && fabsf(hcVel) > 0.004f) {   // lancio: il rullo continua per inerzia
      hcFlingCol = col; hcFlingVel = hcVel; hcFlingLastMs = now;
    } else {
      hcSnap(hcField, col);
      drawHomeCoordsScreen();
    }
    return;
  }
  hcDragDecided = false; hcTouchId = 0;
  if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
    // swipe dx/sx: esce senza salvare e torna alla scelta del formato
    hcFlingCol = -1;
    currentScreen = HOME_COORDS_FMT_SCREEN;
    drawHomeCoordsFormatScreen();
    playTone(1500, 80);
    return;
  }
  if (!tapValid) return;
  if (startX >= 64 && startX <= 136 && startY >= 38 && startY <= 88) {
    hcNeg[hcField] = !hcNeg[hcField];
    drawHomeCoordsScreen();
    playTone(1200, 50);
  } else if (startX >= 214 && startX <= 300 && startY >= 38 && startY <= 88) {
    if (hcFlingCol >= 0) { hcSnap(hcField, hcFlingCol); hcFlingCol = -1; }
    hcField ^= 1;
    drawHomeCoordsScreen();
    playTone(1200, 50);
  } else if (startX >= 115 && startX <= 245 && startY >= 278 && startY <= 345) {
    for (int f = 0; f < 2; ++f) for (int c = 0; c < 3; ++c) hcSnap(f, c);
    hcFlingCol = -1;
    const double lat = hcDegrees(0), lon = hcDegrees(1);
    currentScreen = HOME_FIELD_SCREEN;
    homeConfirmPending = HOME_CONFIRM_NONE;
    applyHomeLocal(true, lat, lon);   // salva e ridisegna HOME FIELD
    if (bleRole == BLE_ROLE_SLAVE) bleQueueHome();   // lo slave la manda al master; il master la invia agli slave
    playTone(1500, 80);
  } else if (startY >= HC_TOUCH_Y0 && startY <= HC_TOUCH_Y1 &&
             (startY < HC_BAND_Y0 || startY > HC_BAND_Y1)) {
    // tocco sopra/sotto la fascia: un passo indietro/avanti
    const int col = hcColumnAt(startX);
    hcPos[hcField][col] += (startY < HC_BAND_Y0) ? -1.0f : 1.0f;
    hcKeepBounded(hcField, col);
    hcSnap(hcField, col);
    drawHomeCoordsScreen();
    playTone(1200, 30);
  }
}

// Inerzia dei rulli: chiamata a ogni giro del loop.
void updateHomeCoordsScreen() {
  if (currentScreen != HOME_COORDS_SCREEN) { hcFlingCol = -1; return; }
  if (hcFlingCol < 0) return;
  const unsigned long now = millis();
  const float dt = (float)(now - hcFlingLastMs);
  if (dt < 1.0f) return;
  hcFlingLastMs = now;
  hcPos[hcField][hcFlingCol] += hcFlingVel * dt;
  hcKeepBounded(hcField, hcFlingCol);
  hcFlingVel *= expf(-dt / 350.0f);   // attrito
  if (fabsf(hcFlingVel) < 0.003f) {
    hcSnap(hcField, hcFlingCol);
    hcFlingCol = -1;
    drawHomeCoordsScreen();
  } else if (now - hcLastDrawMs >= 30UL) {
    drawHomeCoordsScreen();
  }
}

// ========== PIN 1234: tastiera numerica comune (ingresso in PAGES e sblocco degli offset) ==========
// Disegnata direttamente sul display; swipe = indietro (gestito da chi la usa).
static const char *const PAGES_PIN = "1234";
static char pinPadEntry[5] = "";
static bool pinPadWrong = false;
static const int PIN_KEY_X0 = 70, PIN_KEY_Y0 = 100, PIN_KEY_W = 70, PIN_KEY_H = 44, PIN_KEY_DX = 75, PIN_KEY_DY = 50;
static void pinPadReset() { pinPadEntry[0] = '\0'; pinPadWrong = false; }
static const char *pinPadTitle = "PIN";   // titolo della tastiera (PIN o HARD RESET)
static void drawPinPad() {
  lcd.fillScreen(themeBg());
  drawUiVlw(pinPadTitle, 180, 34, MC_DATUM, themeColor(TFT_CYAN), themeBg(), (strlen(pinPadTitle) > 4 ? 0.75f : 0.9f) * THEME_TEXT_SCALE);
  const int n = (int)strlen(pinPadEntry);
  for (int i = 0; i < 4; ++i) {   // un pallino per cifra
    const int x = 135 + i * 30;
    if (i < n) lcd.fillCircle(x, 76, 7, themeColor(TFT_WHITE));
    else { lcd.drawCircle(x, 76, 7, themeColor(TFT_LIGHTGREY)); lcd.drawCircle(x, 76, 6, themeColor(TFT_LIGHTGREY)); }
  }
  static const char *const KEYS[12] = { "1", "2", "3", "4", "5", "6", "7", "8", "9", "C", "0", "OK" };
  for (int k = 0; k < 12; ++k) {
    const int x = PIN_KEY_X0 + (k % 3) * PIN_KEY_DX, y = PIN_KEY_Y0 + (k / 3) * PIN_KEY_DY;
    const uint16_t c = k == 9 ? themeVivid(TFT_RED) : (k == 11 ? themeVivid(TFT_GREEN) : themeColor(TFT_WHITE));
    const uint16_t frame = (k == 9 || k == 11) ? c : themeColor(TFT_DARKGREY);
    lcd.drawRoundRect(x, y, PIN_KEY_W, PIN_KEY_H, 8, frame);
    lcd.drawRoundRect(x + 1, y + 1, PIN_KEY_W - 2, PIN_KEY_H - 2, 7, frame);
    drawUiVlw(KEYS[k], x + PIN_KEY_W / 2, y + PIN_KEY_H / 2 + 1, MC_DATUM, c, themeBg(), 0.75f);
  }
  if (pinPadWrong) drawUiVlw("WRONG PIN", 180, 316, MC_DATUM, themeVivid(TFT_RED), themeBg(), 0.55f);
  drawUiVlw("Swipe to exit", 180, 340, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.5f);
}
// Tocco sulla tastiera: true = PIN giusto (chi chiama ridisegna la sua pagina), altrimenti ridisegna la tastiera.
static bool pinPadTap(int tx, int ty, const char *pin = nullptr) {
  int key = -1;
  for (int k = 0; k < 12 && key < 0; ++k) {
    const int x = PIN_KEY_X0 + (k % 3) * PIN_KEY_DX, y = PIN_KEY_Y0 + (k / 3) * PIN_KEY_DY;
    if (tx >= x - 2 && tx < x + PIN_KEY_W + 3 && ty >= y - 2 && ty < y + PIN_KEY_H + 4) key = k;
  }
  if (key < 0) return false;
  const size_t n = strlen(pinPadEntry);
  if (key == 9) {                       // C: cancella tutto
    pinPadReset();
    playTone(900, 40);
  } else if (key == 11) {               // OK
    if (strcmp(pinPadEntry, pin ? pin : PAGES_PIN) == 0) {
      pinPadReset();
      playTone(1500, 80);
      playTone(2000, 100);
      return true;
    }
    pinPadEntry[0] = '\0';
    pinPadWrong = true;
    playTone(600, 200);
  } else if (n < 4) {
    static const char DIG[12] = { '1', '2', '3', '4', '5', '6', '7', '8', '9', 0, '0', 0 };
    pinPadEntry[n] = DIG[key];
    pinPadEntry[n + 1] = '\0';
    pinPadWrong = false;
    playTone(1200, 30);
  } else {
    playTone(600, 40);                  // gia' 4 cifre
  }
  drawPinPad();
  return false;
}

// ========== ALT OFFSET: taratura dell'offset del sensore altimetro (voce OFFSET della pagina PAGES) ==========
// Pulsanti - e + (metri interi, da -100 a +100 m; METEO OFFSET usa la stessa pagina in gradi interi, +-10).
// Sotto, la quota attuale ricalcolata con l'offset scelto, per confrontarla con la quota nota del
// campo. SAVE salva solo l'offset (namespace NVS "calib", non toccato da "Reset preferences") e lo applica subito.
// Swipe a destra o a sinistra: torna a PAGES senza salvare.
static const int OFS_STEPS_MAX = 100;       // ALT OFFSET: +-100 m a passi di 1 m
static int ofsMode = 0;                     // 0 = ALT OFFSET (metri interi, +-100), 1 = METEO OFFSET (gradi interi, +-10)
static int ofsMax() { return ofsMode == 1 ? 10 : OFS_STEPS_MAX; }
// METEO OFFSET: pulsanti - e + ai lati del valore
static const int MOF_BTN_W = 74, MOF_BTN_H = 64, MOF_BTN_Y = 133, MOF_MINUS_X = 30, MOF_PLUS_X = 256;
static const int OFS_CY = 165;              // riga centrale del rullo
static const int OFS_ROW = 40;
static const int OFS_AREA_Y0 = 82, OFS_AREA_Y1 = 248;
static const int OFS_BAND_Y0 = 141, OFS_BAND_Y1 = 189;
static float ofsPos = 11.0f;                // valore scelto (metri o gradi interi)
static bool ofsDragging = false, ofsDragDecided = false;
static unsigned long ofsTouchId = 0, ofsLastMs = 0, ofsLastDrawMs = 0, ofsFlingLastMs = 0;
static int ofsLastY = 0;
static float ofsVel = 0.0f, ofsFlingVel = 0.0f;
static bool ofsFling = false;
// LOCK: blocca ALT OFFSET e METEO OFFSET (secondo blocco, oltre al PIN di PAGES). Si sblocca con il PIN.
// Salvato in "calib" (come le tarature: resta dopo Reset preferences, aggiornamenti e spegnimento).
static bool offsetsLocked = false;
static bool ofsPinMode = false;             // true = tastiera del PIN al posto della pagina
// METEO OFFSET: tocco sulla scritta "SAVE = ..." per scegliere a mano cosa fa SAVE
// (COLD -> WARM -> OFFSET -> COLD). -1 = scelta automatica (in base al tempo dall'accensione e al calore).
static int ofsCalibSel = -1;
int meteoCalibMode(float rawTemp);
static int meteoSaveMode(float rawT) { return ofsCalibSel >= 0 ? ofsCalibSel : meteoCalibMode(rawT); }
static const int OFS_SAVE_X = 88, OFS_LOCK_X = 198, OFS_LOCK_W = 76, OFS_LOCKED_X = 110, OFS_LOCKED_W = 140;
static void saveOffsetsLock() {
  if (preferences.begin("calib", false)) { preferences.putBool("ofs_lock", offsetsLocked); preferences.end(); }
}
static void drawPadlock(lgfx::LGFXBase &g, int x, int y, uint16_t c) {   // lucchetto: corpo + archetto
  g.fillRoundRect(x - 8, y - 1, 16, 12, 2, c);
  g.fillArc(x, y - 1, 5, 7, 180, 360, c);
  g.fillRect(x - 7, y - 2, 2, 2, c);
  g.fillRect(x + 5, y - 2, 2, 2, c);
}

void loadAltOffset() {
  float v = ALT_OFFSET_DEFAULT_M, t = 0.0f;
  if (preferences.begin("calib", true)) {
    v = preferences.getFloat("alt_off", ALT_OFFSET_DEFAULT_M);
    t = preferences.getFloat("met_off", 0.0f);   // METEO OFFSET
    meteoHeatK = preferences.getFloat("met_k", 0.0f);
    meteoHeatD0 = preferences.getFloat("met_d0", NAN);
    offsetsLocked = preferences.getBool("ofs_lock", false);
    preferences.end();
  }
  if (!isfinite(meteoHeatK)) meteoHeatK = 0.0f;
  meteoHeatK = constrain(meteoHeatK, 0.0f, 3.0f);
  if (!isfinite(v)) v = ALT_OFFSET_DEFAULT_M;
  altOffsetM = constrain(roundf(v), -100.0f, 100.0f);   // metri interi
  if (!isfinite(t)) t = 0.0f;
  meteoTempOffsetC = constrain(roundf(t), -10.0f, 10.0f);   // gradi interi (niente decimi)
}

static void ofsClamp() { ofsPos = constrain(ofsPos, (float)-ofsMax(), (float)ofsMax()); }
static int ofsSteps() { return constrain((int)lroundf(ofsPos), -ofsMax(), ofsMax()); }
static void ofsFmt(int steps, char *out, size_t n) {   // valori interi: gradi (METEO) o metri (ALT)
  snprintf(out, n, steps == 0 ? "0" : "%+d", steps);
}

void drawAltOffsetScreen() {
  ofsLastDrawMs = millis();
  if (ofsPinMode) { drawPinPad(); return; }
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  const uint16_t gold = lgfx::color565(239, 159, 39);
  const uint16_t green = lgfx::color565(29, 158, 117);
  const uint16_t grey = lgfx::color565(170, 170, 170);
  g.fillScreen(TFT_BLACK);
  hcText(g, ofsMode == 1 ? "METEO OFFSET" : "ALT OFFSET", 180, 46, 0.8f, TFT_CYAN, MC_DATUM);

  {
    // Valore intero al centro (gradi o metri), pulsanti - e + ai lati
    const int cy = MOF_BTN_Y + MOF_BTN_H / 2;
    for (int b = 0; b < 2; ++b) {
      const int bx = b == 0 ? MOF_MINUS_X : MOF_PLUS_X;
      const bool en = !offsetsLocked && (b == 0 ? ofsSteps() > -ofsMax() : ofsSteps() < ofsMax());
      const uint16_t c = en ? green : lgfx::color565(68, 68, 68);
      g.drawRoundRect(bx, MOF_BTN_Y, MOF_BTN_W, MOF_BTN_H, 10, c);
      g.drawRoundRect(bx + 1, MOF_BTN_Y + 1, MOF_BTN_W - 2, MOF_BTN_H - 2, 9, c);
      const int mx = bx + MOF_BTN_W / 2;
      g.fillRoundRect(mx - 16, cy - 3, 32, 7, 3, c);                  // segno meno
      if (b == 1) g.fillRoundRect(mx - 3, cy - 16, 7, 32, 3, c);      // segno piu'
    }
    g.fillRoundRect(118, OFS_BAND_Y0 - 4, 124, OFS_BAND_Y1 - OFS_BAND_Y0 + 8, 8, lgfx::color565(26, 26, 26));
    char vb[8], vd[12];
    ofsFmt(ofsSteps(), vb, sizeof(vb));
    if (ofsMode == 1) {
      snprintf(vd, sizeof(vd), "%s*C", vb);                 // '*' = simbolo dei gradi
      hcTextDeg(g, vd, 180, cy, 1.15f, offsetsLocked ? lgfx::color565(200, 200, 200) : (uint16_t)TFT_WHITE);
    } else {
      snprintf(vd, sizeof(vd), "%s m", vb);
      hcText(g, vd, 180, cy, 1.15f, offsetsLocked ? lgfx::color565(200, 200, 200) : (uint16_t)TFT_WHITE, MC_DATUM);
    }
  }
  // Pulsanti in basso: SAVE + LOCK (sbloccato) oppure LOCKED (bloccato: tocco = PIN)
  auto drawBottomButtons = [&]() {
    if (offsetsLocked) {
      g.drawRoundRect(OFS_LOCKED_X, 280, OFS_LOCKED_W, 38, 8, gold);
      g.drawRoundRect(OFS_LOCKED_X + 1, 281, OFS_LOCKED_W - 2, 36, 7, gold);
      drawPadlock(g, OFS_LOCKED_X + 25, 296, gold);
      hcText(g, "LOCKED", OFS_LOCKED_X + 82, 299, 0.62f, gold, MC_DATUM);
    } else {
      g.drawRoundRect(OFS_SAVE_X, 280, 100, 38, 8, gold);
      g.drawRoundRect(OFS_SAVE_X + 1, 281, 98, 36, 7, gold);
      hcText(g, "SAVE", OFS_SAVE_X + 50, 299, 0.75f, gold, MC_DATUM);
      g.drawRoundRect(OFS_LOCK_X, 280, OFS_LOCK_W, 38, 8, grey);
      g.drawRoundRect(OFS_LOCK_X + 1, 281, OFS_LOCK_W - 2, 36, 7, grey);
      drawPadlock(g, OFS_LOCK_X + 18, 296, grey);
      hcText(g, "LOCK", OFS_LOCK_X + 51, 299, 0.45f, grey, MC_DATUM);
    }
  };

  char line[40];
  if (ofsMode == 1) {
    // Temperatura con l'offset del rullo, per confrontarla con un termometro affidabile
    float rawT = NAN, rawH = NAN; bool okT = false;
    if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
      rawT = meteo_temperature_c; rawH = meteo_humidity_percent; okT = meteoValid;
      xSemaphoreGive(sensorDataMutex);
    }
    if (okT && isfinite(rawT)) {
      const float keep = meteoTempOffsetC;
      float ct, ch;
      meteoTempOffsetC = (float)ofsSteps();           // anteprima con il valore scelto
      meteoForceInternal = true;                       // la taratura riguarda il sensore interno
      compensateOutsideAirReading(rawT, rawH, ct, ch);
      meteoForceInternal = false;
      meteoTempOffsetC = keep;
      snprintf(line, sizeof(line), "TEMP: %d C  RH %d%%", (int)lroundf(ct), (int)lroundf(ch));
    } else {
      strcpy(line, "TEMP: no sensor");
    }
    {   // cosa fa SAVE adesso, e a che punto e' la taratura (freddo / caldo)
      const int mode = meteoSaveMode(okT ? rawT : NAN);
      const char *hint = mode == 1 ? "SAVE = COLD CALIBRATION" : (mode == 2 ? "SAVE = WARM CALIBRATION" : "SAVE = OFFSET");
      if (offsetsLocked) hcText(g, "LOCKED - tap to unlock", 180, 80, 0.45f, gold, MC_DATUM);
      else {
        const uint16_t hc = mode == 0 ? lgfx::color565(160, 160, 160) : (uint16_t)TFT_YELLOW;
        hcText(g, hint, 180, 72, 0.5f, hc, MC_DATUM);
        // frecce ai lati: la scritta si tocca per cambiare (piene = scelta a mano)
        const uint16_t ac = ofsCalibSel >= 0 ? hc : lgfx::color565(90, 90, 90);
        g.fillTriangle(44, 72, 52, 66, 52, 78, ac);
        g.fillTriangle(316, 72, 308, 66, 308, 78, ac);
      }
      const float chip = chipTempC;
      char st[40];
      bool ready = false;
      if (!isfinite(chip)) strcpy(st, "chip not available");
      else if (!(okT && isfinite(rawT))) strcpy(st, "sensor not available");
      else if (mode == 1) {
        if (millis() < 5UL * 60000UL) {
          const unsigned long left = 5UL * 60000UL - millis();
          snprintf(st, sizeof(st), "cold: %lu min left", (left + 59999UL) / 60000UL);
        } else strcpy(st, "cold: only if the instrument is cold");   // scelta a mano dopo i 5 minuti
        ready = true;
      } else if (!isfinite(meteoHeatD0)) strcpy(st, "cold calibration missing");
      else {
        const float need = 2.0f - ((chip - rawT) - meteoHeatD0);
        if (need > 0.0f) snprintf(st, sizeof(st), "warm: %.1f C to go", need);
        else { strcpy(st, "cold OK - warm ready"); ready = true; }
      }
      if (!offsetsLocked) hcText(g, st, 180, 98, 0.4f, ready ? (uint16_t)TFT_YELLOW : lgfx::color565(255, 140, 0), MC_DATUM);
      char rw[40];
      if (oatValid && okT && isfinite(rawT)) snprintf(rw, sizeof(rw), "sensor %d C   ext probe %d C", (int)lroundf(rawT), (int)lroundf(oatTempC));
      else if (isfinite(chip) && okT && isfinite(rawT)) snprintf(rw, sizeof(rw), "chip %d C   sensor %d C", (int)lroundf(chip), (int)lroundf(rawT));
      else if (okT && isfinite(rawT)) snprintf(rw, sizeof(rw), "chip --   sensor %d C", (int)lroundf(rawT));
      else strcpy(rw, "chip --   sensor --");
      hcText(g, rw, 180, 232, 0.4f, lgfx::color565(150, 150, 150), MC_DATUM);
    }
    hcText(g, line, 180, 262, 0.62f, green, MC_DATUM);
    drawBottomButtons();
    if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
    return;
  }

  // Quota attuale con l'offset del rullo (e quella con l'offset salvato)
  float raw = NAN; bool ok = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
    raw = rawAltitudeShared; ok = rawAltitudeValid;
    xSemaphoreGive(sensorDataMutex);
  }
  if (ok && isfinite(raw)) {
    const float alt = raw - altOffsetM + (float)ofsSteps();
    if (altitudeUnitsIsMeters) snprintf(line, sizeof(line), "ALT: %d mt", (int)lroundf(alt));
    else snprintf(line, sizeof(line), "ALT: %d ft", (int)lroundf(alt * 3.28084f));
  } else {
    strcpy(line, bleRole == BLE_ROLE_SLAVE ? "SLAVE: set on master" : "ALT: no sensor");
  }
  hcText(g, line, 180, 262, 0.62f, green, MC_DATUM);
  char saved[24], sv[10];
  ofsFmt((int)lroundf(altOffsetM), sv, sizeof(sv));
  snprintf(saved, sizeof(saved), "saved %s m", sv);
  hcText(g, saved, 180, 336, 0.42f, grey, MC_DATUM);

  if (offsetsLocked) hcText(g, "LOCKED - tap to unlock", 180, 80, 0.45f, gold, MC_DATUM);
  drawBottomButtons();
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

void openAltOffsetScreen() {
  ofsMode = 0;
  ofsPinMode = false;
  ofsPos = (float)lroundf(altOffsetM);
  ofsClamp();
  ofsDragging = ofsDragDecided = ofsFling = false;
  ofsTouchId = 0;
  currentScreen = ALT_OFFSET_SCREEN;
  drawAltOffsetScreen();
}
// METEO OFFSET (PAGES 2/2): pulsanti - e +, gradi interi tra -10 e +10
void openMeteoOffsetScreen() {
  ofsMode = 1;
  ofsPinMode = false;
  ofsCalibSel = -1;   // si riparte dalla scelta automatica
  ofsPos = (float)lroundf(meteoTempOffsetC);
  ofsClamp();
  ofsDragging = ofsDragDecided = ofsFling = false;
  ofsTouchId = 0;
  currentScreen = ALT_OFFSET_SCREEN;
  drawAltOffsetScreen();
}

void altOffsetTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != ALT_OFFSET_SCREEN) return;
  const unsigned long now = millis();
  if (touchId != ofsTouchId) {
    ofsTouchId = touchId;
    ofsDragging = ofsDragDecided = false;
    ofsLastY = startY; ofsLastMs = now; ofsVel = 0.0f;
    if (ofsFling) { ofsFling = false; ofsPos = (float)ofsSteps(); drawAltOffsetScreen(); }
    return;
  }
  if (!ofsDragDecided) {
    const int dx = abs(x - startX), dy = abs(y - startY);
    if (dx < 8 && dy < 8) return;
    ofsDragDecided = true;
    ofsDragging = false;   // niente rullo: si usano i pulsanti - e +
  }
  if (!ofsDragging) return;
  const int d = y - ofsLastY;
  const unsigned long dt = max(1UL, now - ofsLastMs);
  ofsLastY = y; ofsLastMs = now;
  if (d == 0) return;
  const float speed = fabsf((float)d) / (float)dt;
  const float gain = constrain(1.0f + (speed - 0.4f) * 4.0f, 1.0f, 6.0f);
  const float steps = -(float)d / (float)OFS_ROW * gain;   // dito verso l'alto = valori piu' grandi
  ofsPos += steps;
  ofsClamp();
  ofsVel = 0.6f * ofsVel + 0.4f * (steps / (float)dt);
  if (now - ofsLastDrawMs >= 30UL) drawAltOffsetScreen();
}

void altOffsetTouchRelease(bool tapValid, int diffX, int diffY, int startX, int startY) {
  const unsigned long now = millis();
  if (ofsDragging) {
    ofsDragging = ofsDragDecided = false; ofsTouchId = 0;
    if (now - ofsLastMs < 80UL && fabsf(ofsVel) > 0.004f) {
      ofsFling = true; ofsFlingVel = ofsVel; ofsFlingLastMs = now;
    } else {
      ofsPos = (float)ofsSteps();
      drawAltOffsetScreen();
    }
    return;
  }
  ofsDragDecided = false; ofsTouchId = 0;
  if (ofsPinMode) {   // tastiera del PIN: swipe = torna alla pagina dell'offset, PIN giusto = sblocca
    if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
      ofsPinMode = false;
      playTone(1500, 80);
      drawAltOffsetScreen();
    } else if (tapValid && pinPadTap(startX, startY)) {
      ofsPinMode = false;
      offsetsLocked = false;
      saveOffsetsLock();
      drawAltOffsetScreen();
    }
    return;
  }
  if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
    ofsFling = false;
    currentScreen = PAGE_SELECT_SCREEN;   // esce senza salvare
    drawPageSelectScreen();
    playTone(1500, 80);
    return;
  }
  if (!tapValid) return;
  if (offsetsLocked) {   // bloccato: solo il tocco su LOCKED (o sulla scritta in alto) apre il PIN
    if ((startX >= OFS_LOCKED_X - 8 && startX <= OFS_LOCKED_X + OFS_LOCKED_W + 8 && startY >= 272 && startY <= 328) ||
        (startY >= 62 && startY <= 100)) {
      pinPadReset();
      pinPadTitle = "PIN";
      ofsPinMode = true;
      playTone(1500, 80);
      drawAltOffsetScreen();
    } else {
      playTone(600, 60);
    }
    return;
  }
  if (ofsMode == 1 && startY >= 56 && startY <= 90 && startX >= 36 && startX <= 324) {   // scritta SAVE = ...: cambia scelta
    float rawT = NAN;
    if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
      if (meteoValid) rawT = meteo_temperature_c;
      xSemaphoreGive(sensorDataMutex);
    }
    const int m = meteoSaveMode(rawT);
    ofsCalibSel = m == 1 ? 2 : (m == 2 ? 0 : 1);   // COLD -> WARM -> OFFSET -> COLD
    playTone(1200, 40);
    drawAltOffsetScreen();
    return;
  }
  if (startX >= OFS_LOCK_X - 4 && startX <= OFS_LOCK_X + OFS_LOCK_W + 10 && startY >= 274 && startY <= 326) {
    offsetsLocked = true;   // LOCK: blocca subito ALT OFFSET e METEO OFFSET
    saveOffsetsLock();
    playTone(1200, 80);
    drawAltOffsetScreen();
    return;
  }
  if (startX >= OFS_SAVE_X - 5 && startX < OFS_LOCK_X - 4 && startY >= 274 && startY <= 326) {
    ofsFling = false;
    ofsPos = (float)ofsSteps();
    if (ofsMode == 1) {
      const float t = (float)ofsSteps();
      float rawT = NAN;
      if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
        if (meteoValid) rawT = meteo_temperature_c;
        xSemaphoreGive(sensorDataMutex);
      }
      int mode = meteoSaveMode(rawT);
      if (mode == 1 && !(isfinite(chipTempC) && isfinite(rawT))) mode = 0;   // senza chip o sensore: solo offset
      if (mode == 2) {   // a caldo serve la taratura a freddo e lo strumento scaldato di almeno mezzo grado
        const float dd0 = (isfinite(chipTempC) && isfinite(rawT) && isfinite(meteoHeatD0)) ? (chipTempC - rawT) - meteoHeatD0 : NAN;
        if (!isfinite(dd0) || dd0 < 0.5f) { playTone(600, 250); drawAltOffsetScreen(); return; }
      }
      if (mode == 2) {
        // a caldo: l'offset a freddo resta, si calcola quanto pesa il calore interno.
        // La temperatura vera e' quella mostrata dal rullo: rawT + t - correzione attuale.
        const float target = rawT + t - meteoHeatCorrection(rawT);
        const float dd = (chipTempC - rawT) - meteoHeatD0;
        meteoHeatK = constrain((rawT + meteoTempOffsetC - target) / dd, 0.0f, 3.0f);
      } else {
        meteoTempOffsetC = t;   // offset (a freddo o semplice); vale subito per tutte le stime meteo
        if (mode == 1) meteoHeatD0 = chipTempC - rawT;   // a freddo: riferimento del chip
      }
      if (preferences.begin("calib", false)) {
        preferences.putFloat("met_off", meteoTempOffsetC);
        preferences.putFloat("met_k", meteoHeatK);
        preferences.putFloat("met_d0", meteoHeatD0);
        preferences.end();
      }
      AVEA_LOGF("METEO: modo %d, offset %+.1f C, k %.2f, d0 %.1f\n", mode, meteoTempOffsetC, meteoHeatK, meteoHeatD0);
      playTone(1500, 120);
      currentScreen = PAGE_SELECT_SCREEN;   // salvato: torna subito a PAGES 2/2
      pageSelectSubPage = 1;
      drawPageSelectScreen();
      return;
    }
    const float v = (float)ofsSteps();
    if (preferences.begin("calib", false)) {
      preferences.putFloat("alt_off", v);
      preferences.end();
    }
    altOffsetM = v;   // il sensorTask lo usa dalla prossima lettura (con reset del vario)
    AVEA_LOGF("ALT OFFSET salvato: %+.1f m\n", v);
    playTone(1500, 120);
    drawAltOffsetScreen();
  } else {   // - e + (zona di tocco un po' piu' larga dei pulsanti)
    const bool minus = startX >= MOF_MINUS_X - 12 && startX <= MOF_MINUS_X + MOF_BTN_W + 10 &&
                       startY >= MOF_BTN_Y - 14 && startY <= MOF_BTN_Y + MOF_BTN_H + 14;
    const bool plus = startX >= MOF_PLUS_X - 10 && startX <= MOF_PLUS_X + MOF_BTN_W + 12 &&
                      startY >= MOF_BTN_Y - 14 && startY <= MOF_BTN_Y + MOF_BTN_H + 14;
    if (!minus && !plus) return;
    const int before = ofsSteps();
    ofsPos = (float)(before + (plus ? 1 : -1));
    ofsClamp();
    drawAltOffsetScreen();
    playTone(ofsSteps() != before ? 1200 : 600, 30);
  }
}

// Inerzia del rullo e aggiornamento della quota mostrata: chiamata a ogni giro del loop.
void updateAltOffsetScreen() {
  if (currentScreen != ALT_OFFSET_SCREEN) { ofsFling = false; return; }
  const unsigned long now = millis();
  if (ofsFling) {
    const float dt = (float)(now - ofsFlingLastMs);
    if (dt < 1.0f) return;
    ofsFlingLastMs = now;
    ofsPos += ofsFlingVel * dt;
    ofsFlingVel *= expf(-dt / 300.0f);
    if (ofsPos <= -ofsMax() || ofsPos >= ofsMax()) ofsFlingVel = 0.0f;
    ofsClamp();
    if (fabsf(ofsFlingVel) < 0.003f) { ofsFling = false; ofsPos = (float)ofsSteps(); drawAltOffsetScreen(); }
    else if (now - ofsLastDrawMs >= 30UL) drawAltOffsetScreen();
  } else if (!ofsDragging && !ofsPinMode && now - ofsLastDrawMs >= 500UL) {
    drawAltOffsetScreen();   // quota attuale aggiornata due volte al secondo
  }
}

// ========== FUEL ESTIMATE: stima del carburante (pagina nel giro, attivabile da PAGES 2/2) ==========
// Lo strumento NON legge il serbatoio: stima i litri da tempo di volo x consumo orario.
// - I litri scendono solo con il timer automatico in volo (decollo -> atterraggio).
// - Al decollo si toglie una volta il consumo fisso a terra (rullaggio + prova motore, GND L).
// - Riserva in minuti al consumo impostato: sotto la riserva arco rosso e avviso RESERVE da ogni pagina.
// - Tocco al centro: FUEL SETUP (1/2: LITERS, L/H, RES MIN; 2/2: TANK, GND L; clic su SETUP per cambiare).
// - Valori nel namespace NVS "fuel" (non toccato da "Reset preferences"); i litri si salvano ogni minuto in volo.
// - In demo il carburante e' simulato (parte poco sopra la riserva, consumo x10) e i valori veri non cambiano.
static int fuelTankL = 60;                 // capacita' serbatoio (l)
static float fuelLiters = 60.0f;           // litri stimati a bordo
static float fuelRateLph = 15.0f;          // consumo orario
static int fuelReserveMin = 30;            // riserva (minuti al consumo impostato)
static float fuelGroundL = 1.5f;           // consumo fisso a terra, tolto al decollo
static uint32_t fuelSinceFullMs = 0;       // tempo di volo dall'ultimo FULL
static bool fuelReserveAlerted = false;
static ScreenState fuelAlertPrevious = MAIN_ALT_SCREEN;
static unsigned long fuelLastDrawMs = 0;
static bool fuelDemoActive = false;
static float fuelDemoLiters = 0.0f;
static const uint16_t FUEL_GREEN = 0x1CEE;   // ~ (29,158,117)
static const uint16_t FUEL_GOLD = 0xECE4;    // ~ (239,159,39)

static float fuelReserveLiters() { return fuelRateLph * (float)fuelReserveMin / 60.0f; }
static float fuelShownLiters() { return fuelDemoActive ? fuelDemoLiters : fuelLiters; }

void loadFuelSettings() {
  if (!preferences.begin("fuel", true)) return;
  fuelTankL = constrain(preferences.getInt("tank", 60), 5, 200);
  fuelLiters = constrain(preferences.getFloat("liters", (float)fuelTankL), 0.0f, (float)fuelTankL);
  fuelRateLph = constrain(preferences.getFloat("lph", 15.0f), 2.0f, 60.0f);
  fuelReserveMin = constrain(preferences.getInt("resmin", 30), 0, 90);
  fuelGroundL = constrain(preferences.getFloat("gnd", 1.5f), 0.0f, 5.0f);
  fuelSinceFullMs = preferences.getULong("sincefull", 0);
  preferences.end();
  if (!isfinite(fuelLiters)) fuelLiters = (float)fuelTankL;
}
// Apre il namespace "fuel" in scrittura; se Preferences risulta ancora aperto lo chiude e riprova.
static bool fuelPrefsBegin() {
  if (preferences.begin("fuel", false)) return true;
  preferences.end();
  const bool ok = preferences.begin("fuel", false);
  if (!ok) AVEA_LOGLN("FUEL: impossibile aprire NVS \"fuel\" per il salvataggio");
  return ok;
}
static void saveFuelLiters() {
  if (!fuelPrefsBegin()) return;
  preferences.putFloat("liters", fuelLiters);
  preferences.putULong("sincefull", fuelSinceFullMs);
  preferences.end();
}
static void saveFuelSettings() {
  if (!fuelPrefsBegin()) return;
  preferences.putInt("tank", fuelTankL);
  preferences.putFloat("lph", fuelRateLph);
  preferences.putInt("resmin", fuelReserveMin);
  preferences.putFloat("gnd", fuelGroundL);
  preferences.end();
  saveFuelLiters();
}

// Valori FUEL ricevuti via BLE (dallo slave sul master, dal master sullo slave). Salva solo se cambiano.
// liters < 0 = non toccare i litri.
void fuelApplyShared(int tank, float lph, int resMin, float gnd, float liters, bool fullPressed) {
  tank = constrain(tank, 5, 200);
  lph = constrain(lph, 2.0f, 60.0f);
  resMin = constrain(resMin, 0, 90);
  gnd = constrain(gnd, 0.0f, 5.0f);
  const bool settingsChanged = tank != fuelTankL || fabsf(lph - fuelRateLph) > 0.01f ||
                               resMin != fuelReserveMin || fabsf(gnd - fuelGroundL) > 0.01f;
  bool litersChanged = false;
  if (liters >= 0.0f) {
    liters = constrain(liters, 0.0f, (float)tank);
    litersChanged = fabsf(liters - fuelLiters) >= 0.05f;
    if (litersChanged) fuelLiters = liters;
  }
  if (fullPressed) fuelSinceFullMs = 0;
  if (settingsChanged) {
    fuelTankL = tank; fuelRateLph = lph; fuelReserveMin = resMin; fuelGroundL = gnd;
    if (fuelLiters > (float)fuelTankL) fuelLiters = (float)fuelTankL;
    saveFuelSettings();
    AVEA_LOGF("FUEL (BLE): %.1f l, %.1f l/h, riserva %d min, serbatoio %d l, terra %.1f l\n",
                  fuelLiters, fuelRateLph, fuelReserveMin, fuelTankL, fuelGroundL);
  } else if ((litersChanged || fullPressed) && !automaticFlightRunning) {
    saveFuelLiters();   // a terra (cambio da setup): salva; in volo i litri si salvano gia' ogni minuto
  }
  if (litersChanged || settingsChanged) fuelReserveAlerted = fuelLiters <= fuelReserveLiters() && !automaticFlightRunning;
  if ((settingsChanged || litersChanged) && (currentScreen == FUEL_SCREEN || currentScreen == FUEL_RESERVE_SCREEN)) drawFuelScreen();
}

// Arco del serbatoio: centro del display, raggio medio FUEL_ARC_R, da 135° a 45° passando per l'alto (270°).
// Bordo esterno sul bordo del display (raggio 179). frac = 0..1 della capacita'. Angoli LovyanGFX: 0° a ore 3, senso orario.
static const int FUEL_ARC_R = 169, FUEL_ARC_HALF = 10;   // arco da 159 a 179: a contatto con il bordo del display
static void fuelArc(lgfx::LGFXBase &g, float rIn, float rOut, float frac0, float frac1, uint16_t c) {
  float a0 = 135.0f + 270.0f * constrain(frac0, 0.0f, 1.0f);
  float a1 = 135.0f + 270.0f * constrain(frac1, 0.0f, 1.0f);
  if (a1 - a0 < 0.3f) return;
  if (a1 <= 360.0f) g.fillArc(180, 180, rIn, rOut, a0, a1, c);
  else if (a0 >= 360.0f) g.fillArc(180, 180, rIn, rOut, a0 - 360.0f, a1 - 360.0f, c);
  else { g.fillArc(180, 180, rIn, rOut, a0, 360.0f, c); g.fillArc(180, 180, rIn, rOut, 0.0f, a1 - 360.0f, c); }
}
static void fuelArcPoint(float frac, int &x, int &y) {
  const float a = (135.0f + 270.0f * constrain(frac, 0.0f, 1.0f)) * DEG_TO_RAD;
  x = 180 + (int)lroundf((float)FUEL_ARC_R * cosf(a));
  y = 180 + (int)lroundf((float)FUEL_ARC_R * sinf(a));
}
static void fuelArcCap(lgfx::LGFXBase &g, float frac, uint16_t c) {   // estremita' arrotondata
  int x, y; fuelArcPoint(frac, x, y);
  g.fillCircle(x, y, FUEL_ARC_HALF, c);
}

// Segno "circa" (≈) disegnato a mano: due onde una sopra l'altra, centrate in (cx, cy).
// I caratteri del display non hanno il simbolo, quindi niente testo.
static void fuelApproxSign(lgfx::LGFXBase &g, int cx, int cy, int w, int amp, int gapY, int thick, uint16_t c) {
  for (int row = 0; row < 2; ++row) {
    const int yc = cy + (row == 0 ? -gapY / 2 : gapY / 2);
    int px = cx - w / 2, py = yc;
    for (int k = 1; k <= 12; ++k) {
      const float t = (float)k / 12.0f;
      const int x = cx - w / 2 + (int)lroundf(t * w);
      const int y = yc - (int)lroundf(amp * sinf(t * 2.0f * PI));
      drawThickLineOn(g, px, py, x, y, c, thick);
      px = x; py = y;
    }
  }
}
// Larghezza dei testi come li scrivono qfeBigNumber (numero grande) e hcText (carattere UI)
static int fuelBigNumberWidth(lgfx::LGFXBase &g, const char *txt, float scale, int maxW) {
  const bool loaded = vlwSet(g, VarioNumbers80);
  const float base = loaded ? 1.0f : 0.8f;
  if (!loaded) g.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
  g.setTextSize(scale * base, scale * base);
  int w = g.textWidth(txt);
  if (w > maxW) w = maxW;
  if (loaded) vlwRelease(g);
  return w;
}
static int fuelUiWidth(lgfx::LGFXBase &g, const char *txt, float scale) {
  const bool loaded = vlwSet(g, UiFont32);
  if (loaded) g.setTextSize(scale, scale);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(scale * 1.4f, scale * 1.4f); }
  const int w = g.textWidth(txt);
  if (loaded) vlwRelease(g);
  return w;
}

void drawFuelScreen() {
  fuelLastDrawMs = millis();
  const bool alert = (currentScreen == FUEL_RESERVE_SCREEN);
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  const uint16_t grey = lgfx::color565(170, 170, 170);
  const uint16_t dark = lgfx::color565(51, 51, 51);
  const uint16_t red = themeVivid(TFT_RED);   // rosso pieno, come gli archi rossi (prima era un rosso tendente al rosa)
  const float liters = fuelShownLiters();
  const float resL = fuelReserveLiters();
  const bool inReserve = liters <= resL + 0.01f;
  const bool nearReserve = !inReserve && liters <= resL + fuelRateLph * 0.25f;   // ultimi 15 min prima della riserva
  const uint16_t arcColor = inReserve ? red : themeColor(TFT_GREEN);   // stesso verde di "TIMER: OFF" (pagina ICT); rosso in riserva
  (void)nearReserve;
  const float frac = constrain(liters / (float)fuelTankL, 0.0f, 1.0f);
  g.fillScreen(TFT_BLACK);
  if (alert) { g.drawCircle(180, 180, 179, red); g.drawCircle(180, 180, 178, red); g.drawCircle(180, 180, 177, red); }

  // Arco: fondo grigio; il carburante e' verde e la parte della riserva (dall'estremita' R) e' rossa,
  // nello stesso arco. Trattino nero al confine riserva/verde, come sugli archi GS.
  const int rIn = FUEL_ARC_R - FUEL_ARC_HALF, rOut = FUEL_ARC_R + FUEL_ARC_HALF;
  const float resFrac = constrain(resL / (float)fuelTankL, 0.0f, 1.0f);
  fuelArc(g, rIn, rOut, 0.0f, 1.0f, dark);
  fuelArc(g, rIn - 7, rIn - 6, 0.0f, 1.0f, lgfx::color565(154, 154, 154));   // arco fisso interno sottile
  // Scala della riserva solo in volo (o nell'avviso): a terra l'arco resta quello normale, perche' con il dito
  // sull'arco si regolano i litri sull'intero serbatoio.
  const bool reserveMode = inReserve && (automaticFlightRunning || alert);
  if (reserveMode) {
    // In riserva tutto l'arco diventa la scala della riserva: da E (vuoto) a R (inizio riserva).
    // Il rosso scende in proporzione ai litri (cioe' ai minuti) di riserva rimasti; la parte consumata e' grigia.
    const float rFrac = resL > 0.01f ? constrain(liters / resL, 0.0f, 1.0f) : 0.0f;
    fuelArcCap(g, 0.0f, rFrac > 0.003f ? red : dark);
    fuelArcCap(g, 1.0f, rFrac > 0.997f ? red : dark);
    if (rFrac > 0.003f) { fuelArc(g, rIn, rOut, 0.0f, rFrac, red); fuelArcCap(g, rFrac, red); }
  } else {
  const bool startColored = frac > 0.003f;
  const bool endColored = frac > 0.997f;
  fuelArcCap(g, 0.0f, startColored ? red : dark);
  fuelArcCap(g, 1.0f, endColored ? arcColor : dark);
  if (startColored) {
    const float redEnd = min(frac, resFrac);
    fuelArc(g, rIn, rOut, 0.0f, redEnd, red);                                  // riserva
    if (frac > resFrac) fuelArc(g, rIn, rOut, resFrac, frac, arcColor);         // sopra la riserva
    fuelArcCap(g, frac, frac > resFrac ? arcColor : red);
  }
  if (resFrac > 0.003f && resFrac < 0.997f) {   // confine della riserva
    const float a = (135.0f + 270.0f * resFrac) * DEG_TO_RAD;
    const int x0 = 180 + (int)lroundf((rIn - 1) * cosf(a)), y0 = 180 + (int)lroundf((rIn - 1) * sinf(a));
    const int x1 = 180 + (int)lroundf((rOut + 1) * cosf(a)), y1 = 180 + (int)lroundf((rOut + 1) * sinf(a));
    drawThickLineOn(g, x0, y0, x1, y1, TFT_BLACK, 4);
  }
  }   // fine arco normale
  // R (rosso) ed F (nero) grandi, su un disco bianco alle estremita' dell'arco (dentro il display).
  // In riserva: E (vuoto) a sinistra e R (inizio riserva) a destra, entrambe rosse.
  {
    const float a0 = 135.0f * DEG_TO_RAD, a1 = 45.0f * DEG_TO_RAD;
    const int rr = FUEL_ARC_R - 3, disc = 13;
    const int rx = 180 + (int)lroundf(rr * cosf(a0)), ry = 180 + (int)lroundf(rr * sinf(a0));
    const int fx = 180 + (int)lroundf(rr * cosf(a1)), fy = 180 + (int)lroundf(rr * sinf(a1));
    g.fillCircle(rx, ry, disc, TFT_WHITE);
    g.fillCircle(fx, fy, disc, TFT_WHITE);
    hcText(g, reserveMode ? "E" : "R", rx, ry + 1, 0.62f, red, MC_DATUM);
    hcText(g, reserveMode ? "R" : "F", fx, fy + 1, 0.62f, reserveMode ? red : (uint16_t)TFT_BLACK, MC_DATUM);
  }

  // Titolo: stesso colore del titolo GPS INFO, piu' grande e piu' in alto, dentro l'arco
  // In riserva il titolo e' rosso e lampeggia (mezzo secondo acceso, mezzo spento), come LOW GS.
  if (!reserveMode) hcText(g, "FUEL ESTIMATE", 180, 69, 0.72f, themeColor(TFT_CYAN), MC_DATUM);
  else if ((millis() / 500UL) % 2UL == 0UL) hcText(g, "FUEL ESTIMATE", 180, 69, 0.72f, red, MC_DATUM);

  // Litri: stesso carattere e grandezza del valore QFE (VarioNumbers80, scala 1.14); rosso in riserva
  char buf[32];
  snprintf(buf, sizeof(buf), "%d", (int)floorf(liters + 0.0001f));
  {   // "≈ 38": il segno circa ricorda che e' una stima
    const uint16_t c = inReserve ? red : (uint16_t)TFT_WHITE;
    const int wN = fuelBigNumberWidth(g, buf, 1.14f, 200), wA = 34, gap = 12;
    const int x0 = 180 - (wA + gap + wN) / 2;
    fuelApproxSign(g, x0 + wA / 2, 140, wA, 4, 14, 4, c);
    qfeBigNumber(g, buf, x0 + wA + gap + wN / 2, 140, 1.14f, 200, c, TFT_BLACK);
  }
  hcText(g, "liters", 180, 196, 0.5f, grey, MC_DATUM);
  // Autonomia: 30% piu' grande di prima (0.85 -> 1.1)
  const float hours = fuelRateLph > 0.1f ? liters / fuelRateLph : 0.0f;
  const int totMin = (int)(hours * 60.0f);
  snprintf(buf, sizeof(buf), "%dh %02dm", totMin / 60, totMin % 60);
  {   // "≈ 2h 32m"; rosso in riserva, come i litri
    const uint16_t c = inReserve ? red : FUEL_GOLD;
    const int wT = fuelUiWidth(g, buf, 1.1f), wA = 22, gap = 9;
    const int x0 = 180 - (wA + gap + wT) / 2;
    fuelApproxSign(g, x0 + wA / 2, 230, wA, 3, 9, 3, c);
    hcText(g, buf, x0 + wA + gap + wT / 2, 230, 1.1f, c, MC_DATUM);
  }
  hcText(g, "endurance", 180, 257, 0.42f, grey, MC_DATUM);

  if (alert || inReserve) {
    g.fillRoundRect(108, 270, 144, 39, 9, red);   // stessa misura del riquadro l/h
    hcText(g, "RESERVE", 180, 289, 0.72f, TFT_BLACK, MC_DATUM);
    // Distanza e tempo per HOME, se impostata
    double lat = 0, lon = 0; float gs = 0; bool ok = false;
    if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
      lat = gps_latitude; lon = gps_longitude; gs = gps_ground_speed_kmh; ok = gpsDataValid;
      xSemaphoreGive(sensorDataMutex);
    }
    if (ok && homePositionValid) {
      const double km = 6371.0 * lubberAngDist(lat * DEG_TO_RAD, lon * DEG_TO_RAD,
                                              homeLatitude * DEG_TO_RAD, homeLongitude * DEG_TO_RAD);
      if (gs > 10.0f) {
        const int m = (int)(km / gs * 60.0);
        snprintf(buf, sizeof(buf), "HOME %.0f km - %dh %02dm", km, m / 60, m % 60);
      } else {
        snprintf(buf, sizeof(buf), "HOME %.0f km", km);
      }
      hcText(g, buf, 180, 326, 0.42f, FUEL_GOLD, MC_DATUM);
    } else if (alert) {
      hcText(g, "Tap to close", 180, 326, 0.42f, grey, MC_DATUM);
    }
  } else {
    // Riquadro l/h 30% piu' grande (110x30 -> 144x39, testo 0.52 -> 0.68)
    g.drawRoundRect(108, 270, 144, 39, 9, lgfx::color565(85, 85, 85));
    g.drawRoundRect(109, 271, 142, 37, 8, lgfx::color565(85, 85, 85));
    snprintf(buf, sizeof(buf), "%.1f l/h", fuelRateLph);
    hcText(g, buf, 180, 289, 0.68f, TFT_WHITE, MC_DATUM);
  }
  if (fuelDemoActive) hcText(g, "DEMO", 180, 344, 0.38f, grey, MC_DATUM);
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

// ---- FUEL SETUP: rulli come le pagine COORDS e OFFSET ----
// Ogni rullo: valore = min + indice * step (indice intero, rullo senza giro).
struct FuelWheel { const char *label; float minV; float step; int count; uint8_t decimals; };
static const FuelWheel FUEL_WHEELS[2][3] = {
  { {"LITERS", 0.0f, 1.0f, 201, 0}, {"L/H", 2.0f, 0.5f, 117, 1}, {"RES MIN", 0.0f, 5.0f, 19, 0} },
  { {"TANK", 5.0f, 1.0f, 196, 0},   {"GND L", 0.0f, 0.5f, 11, 1}, {nullptr, 0, 0, 0, 0} }
};
static int fuelSetupPage = 0;
static bool fsBootMode = false;   // SET LITERS di accensione (arco come FUEL ESTIMATE), uscita = pagina salvata
static float fsBootLiters = 0.0f;  // litri impostati sull'arco di SET LITERS (salvati solo con SAVE)
static bool fsBootArcArmed = false, fsBootArcMoved = false;
static unsigned long fsBootArcTouchId = 0, fsBootArcLastDrawMs = 0;
static bool fuelArcFracAt(int x, int y, float &frac);
static float fsPos[2][3];
static int fsDragCol = -1;
static bool fsDragDecided = false;
static unsigned long fsTouchId = 0, fsLastMs = 0, fsLastDrawMs = 0, fsFlingLastMs = 0;
static int fsLastY = 0;
static float fsVel = 0.0f, fsFlingVel = 0.0f;
static int fsFlingCol = -1;
static const int FS_CY = 180, FS_ROW = 38, FS_AREA_Y0 = 112, FS_AREA_Y1 = 248, FS_BAND_Y0 = 157, FS_BAND_Y1 = 203;

static int fsCols() { return fuelSetupPage == 0 ? 3 : 2; }
static int fsColX(int c) { return fuelSetupPage == 0 ? 95 + c * 85 : 120 + c * 120; }
static int fsColAt(int x) {
  if (fuelSetupPage == 0) return x < 138 ? 0 : (x < 222 ? 1 : 2);
  return x < 180 ? 0 : 1;
}
static int fsIndex(int p, int c) { return constrain((int)lroundf(fsPos[p][c]), 0, FUEL_WHEELS[p][c].count - 1); }
static float fsValue(int p, int c) { return FUEL_WHEELS[p][c].minV + fsIndex(p, c) * FUEL_WHEELS[p][c].step; }
static void fsSet(int p, int c, float v) {
  fsPos[p][c] = (float)constrain((int)lroundf((v - FUEL_WHEELS[p][c].minV) / FUEL_WHEELS[p][c].step), 0,
                                 FUEL_WHEELS[p][c].count - 1);
}
static void fsClamp(int p, int c) { fsPos[p][c] = constrain(fsPos[p][c], 0.0f, (float)(FUEL_WHEELS[p][c].count - 1)); }
static void fsFmt(int p, int c, int idx, char *out, size_t n) {
  const float v = FUEL_WHEELS[p][c].minV + idx * FUEL_WHEELS[p][c].step;
  if (FUEL_WHEELS[p][c].decimals) snprintf(out, n, "%.1f", v); else snprintf(out, n, "%d", (int)lroundf(v));
}

// SET LITERS (accensione): arco come FUEL ESTIMATE, il dito sull'arco regola i litri (passi da 0.5 l).
static void drawBootLitersScreen(lgfx::LGFXBase &g) {
  const uint16_t grey = lgfx::color565(170, 170, 170);
  const uint16_t dark = lgfx::color565(51, 51, 51);
  const uint16_t red = themeVivid(TFT_RED);
  const uint16_t green = themeColor(TFT_GREEN);
  const float liters = fsBootLiters;
  const float resL = fuelReserveLiters();
  const float frac = constrain(liters / (float)fuelTankL, 0.0f, 1.0f);
  const float resFrac = constrain(resL / (float)fuelTankL, 0.0f, 1.0f);
  const int rIn = FUEL_ARC_R - FUEL_ARC_HALF, rOut = FUEL_ARC_R + FUEL_ARC_HALF;
  g.fillScreen(TFT_BLACK);
  fuelArc(g, rIn, rOut, 0.0f, 1.0f, dark);
  fuelArc(g, rIn - 7, rIn - 6, 0.0f, 1.0f, lgfx::color565(154, 154, 154));   // arco fisso interno sottile
  const bool startColored = frac > 0.003f;
  fuelArcCap(g, 0.0f, startColored ? red : dark);
  fuelArcCap(g, 1.0f, frac > 0.997f ? green : dark);
  if (startColored) {
    fuelArc(g, rIn, rOut, 0.0f, min(frac, resFrac), red);                     // riserva
    if (frac > resFrac) fuelArc(g, rIn, rOut, resFrac, frac, green);          // sopra la riserva
    fuelArcCap(g, frac, frac > resFrac ? green : red);
  }
  if (resFrac > 0.003f && resFrac < 0.997f) {   // confine della riserva
    const float a = (135.0f + 270.0f * resFrac) * DEG_TO_RAD;
    drawThickLineOn(g, 180 + (int)lroundf((rIn - 1) * cosf(a)), 180 + (int)lroundf((rIn - 1) * sinf(a)),
                    180 + (int)lroundf((rOut + 1) * cosf(a)), 180 + (int)lroundf((rOut + 1) * sinf(a)), TFT_BLACK, 4);
  }
  {   // pallino bianco sul punto dei litri: si trascina con il dito
    const float a = (135.0f + 270.0f * frac) * DEG_TO_RAD;
    const int kx = 180 + (int)lroundf(FUEL_ARC_R * cosf(a)), ky = 180 + (int)lroundf(FUEL_ARC_R * sinf(a));
    g.fillCircle(kx, ky, 14, frac > resFrac ? green : red);
    g.fillCircle(kx, ky, 11, TFT_WHITE);
  }
  {   // R ed F su disco bianco alle estremita'
    const float a0 = 135.0f * DEG_TO_RAD, a1 = 45.0f * DEG_TO_RAD;
    const int rr = FUEL_ARC_R - 3, disc = 13;
    const int rx = 180 + (int)lroundf(rr * cosf(a0)), ry = 180 + (int)lroundf(rr * sinf(a0));
    const int fx = 180 + (int)lroundf(rr * cosf(a1)), fy = 180 + (int)lroundf(rr * sinf(a1));
    g.fillCircle(rx, ry, disc, TFT_WHITE);
    g.fillCircle(fx, fy, disc, TFT_WHITE);
    hcText(g, "R", rx, ry + 1, 0.62f, red, MC_DATUM);
    hcText(g, "F", fx, fy + 1, 0.62f, TFT_BLACK, MC_DATUM);
  }
  hcText(g, "SET LITERS", 180, 69, 0.72f, themeColor(TFT_CYAN), MC_DATUM);
  char buf[32];
  snprintf(buf, sizeof(buf), "%d", (int)floorf(liters + 0.0001f));
  qfeBigNumber(g, buf, 180, 140, 1.14f, 200, liters <= resL + 0.01f ? red : (uint16_t)TFT_WHITE, TFT_BLACK);
  hcText(g, "liters", 180, 196, 0.5f, grey, MC_DATUM);
  const int totMin = fuelRateLph > 0.1f ? (int)(liters / fuelRateLph * 60.0f) : 0;
  snprintf(buf, sizeof(buf), "%dh %02dm", totMin / 60, totMin % 60);
  hcText(g, buf, 180, 230, 1.1f, FUEL_GOLD, MC_DATUM);
  hcText(g, "endurance", 180, 254, 0.42f, grey, MC_DATUM);
  g.drawRoundRect(74, 268, 100, 34, 8, FUEL_GOLD); g.drawRoundRect(75, 269, 98, 32, 7, FUEL_GOLD);
  hcText(g, "FULL", 124, 285, 0.6f, FUEL_GOLD, MC_DATUM);
  g.drawRoundRect(186, 268, 100, 34, 8, FUEL_GOLD); g.drawRoundRect(187, 269, 98, 32, 7, FUEL_GOLD);
  hcText(g, "SAVE", 236, 285, 0.6f, FUEL_GOLD, MC_DATUM);
  snprintf(buf, sizeof(buf), "TANK %d l  -  2 / 2", fuelTankL);
  hcText(g, buf, 180, 326, 0.42f, lgfx::color565(136, 136, 136), MC_DATUM);
}

void drawFuelSetupScreen() {
  fsLastDrawMs = millis();
  const bool spr = ensureNavigationSprite();
  lgfx::LGFXBase &g = spr ? (lgfx::LGFXBase&)navigationSprite : (lgfx::LGFXBase&)lcd;
  if (fsBootMode) {
    drawBootLitersScreen(g);
    if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
    return;
  }
  const uint16_t grey = lgfx::color565(136, 136, 136);
  g.fillScreen(TFT_BLACK);
  hcText(g, "FUEL ESTIMATE", 180, 58, 0.6f, TFT_CYAN, MC_DATUM);
  hcText(g, fuelSetupPage == 0 ? "SETUP 1/2" : "SETUP 2/2", 180, 84, 0.45f, lgfx::color565(170, 170, 170), MC_DATUM);
  const int p = fuelSetupPage;
  // Nomi dei rulli (LITERS, L/H, RES MIN / TANK, GND L) piu' grandi
  for (int c = 0; c < fsCols(); ++c) hcText(g, FUEL_WHEELS[p][c].label, fsColX(c), 102, 0.52f, lgfx::color565(170, 170, 170), MC_DATUM);
  g.fillRoundRect(40, FS_BAND_Y0, 280, FS_BAND_Y1 - FS_BAND_Y0, 8, lgfx::color565(26, 26, 26));
  g.drawRoundRect(40, FS_BAND_Y0, 280, FS_BAND_Y1 - FS_BAND_Y0, 8, FUEL_GREEN);
  g.drawRoundRect(41, FS_BAND_Y0 + 1, 278, FS_BAND_Y1 - FS_BAND_Y0 - 2, 7, FUEL_GREEN);
  g.setClipRect(0, FS_AREA_Y0, 360, FS_AREA_Y1 - FS_AREA_Y0);
  for (int c = 0; c < fsCols(); ++c) {
    const int base = (int)lroundf(fsPos[p][c]);
    const float frac = fsPos[p][c] - (float)base;
    for (int k = -2; k <= 2; ++k) {
      const int idx = base - k;   // valori piu' grandi sopra
      if (idx < 0 || idx >= FUEL_WHEELS[p][c].count) continue;
      const int y = (int)lroundf(FS_CY + ((float)k + frac) * FS_ROW);
      const int dist = abs(y - FS_CY);
      char buf[10];
      fsFmt(p, c, idx, buf, sizeof(buf));
      const uint16_t col = dist < FS_ROW / 2 ? (uint16_t)TFT_WHITE
                         : (dist < FS_ROW * 3 / 2 ? grey : lgfx::color565(68, 68, 68));
      hcText(g, buf, fsColX(c), y, dist < FS_ROW / 2 ? 1.0f : 0.7f, col, MC_DATUM);
    }
  }
  g.clearClipRect();
  if (p == 0) {   // FULL = litri al pieno (TANK)
    g.drawRoundRect(74, 254, 100, 34, 8, FUEL_GOLD); g.drawRoundRect(75, 255, 98, 32, 7, FUEL_GOLD);
    hcText(g, "FULL", 124, 271, 0.6f, FUEL_GOLD, MC_DATUM);
    g.drawRoundRect(186, 254, 100, 34, 8, FUEL_GOLD); g.drawRoundRect(187, 255, 98, 32, 7, FUEL_GOLD);
    hcText(g, "SAVE", 236, 271, 0.6f, FUEL_GOLD, MC_DATUM);
    const unsigned long m = fuelSinceFullMs / 60000UL;
    char buf[40];
    // Su due righe e piu' grandi: serbatoio, poi tempo di volo dall'ultimo FULL
    snprintf(buf, sizeof(buf), "TANK %d l", (int)lroundf(fsValue(1, 0)));
    hcText(g, buf, 180, 305, 0.55f, lgfx::color565(200, 200, 200), MC_DATUM);
    snprintf(buf, sizeof(buf), "since FULL %luh %02lum", m / 60UL, m % 60UL);
    hcText(g, buf, 180, 327, 0.5f, lgfx::color565(170, 170, 170), MC_DATUM);
  } else {
    g.drawRoundRect(130, 254, 100, 34, 8, FUEL_GOLD); g.drawRoundRect(131, 255, 98, 32, 7, FUEL_GOLD);
    hcText(g, "SAVE", 180, 271, 0.6f, FUEL_GOLD, MC_DATUM);
    hcText(g, "GND L: taxi + run-up", 180, 312, 0.52f, lgfx::color565(170, 170, 170), MC_DATUM);
  }
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}

static bool fsFullPressed = false;   // FULL premuto: al SAVE azzera il tempo "since FULL"
static bool fsFromPages = false;     // setup aperto da PAGES 2/2 (SET): all'uscita torna li'

static void fuelSetupExit() {
  if (fsBootMode) { fsBootMode = false; bootWizardFinish(); return; }   // SET LITERS di accensione
  if (fsFromPages) { currentScreen = PAGE_SELECT_SCREEN; drawPageSelectScreen(); }
  else { currentScreen = FUEL_SCREEN; drawFuelScreen(); }
}

void openFuelSetupFromPages() {
  openFuelSetupScreen();
  fsFromPages = true;
}

void openFuelSetupScreen() {
  if (realFlightActive()) { playTone(600, 120); return; }   // in volo reale il carburante non si imposta
  fsFromPages = false;
  fsSet(0, 0, fuelLiters); fsSet(0, 1, fuelRateLph); fsSet(0, 2, (float)fuelReserveMin);
  fsSet(1, 0, (float)fuelTankL); fsSet(1, 1, fuelGroundL);
  fuelSetupPage = 0; fsDragCol = -1; fsDragDecided = false; fsTouchId = 0; fsFlingCol = -1;
  fsFullPressed = false;
  fsBootLiters = fuelLiters;
  fsBootArcArmed = false; fsBootArcMoved = false; fsBootArcTouchId = 0;
  currentScreen = FUEL_SETUP_SCREEN;
  drawFuelSetupScreen();
}

void fuelSetupTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != FUEL_SETUP_SCREEN) return;
  if (fsBootMode) {   // SET LITERS: dito sull'arco
    if (touchId != fsBootArcTouchId) {
      fsBootArcTouchId = touchId; fsBootArcMoved = false;
      float f0;
      fsBootArcArmed = fuelArcFracAt(startX, startY, f0);
      return;
    }
    if (!fsBootArcArmed) return;
    if (!fsBootArcMoved && abs(x - startX) < 6 && abs(y - startY) < 6) return;
    if (!fsBootArcMoved && arcDragIsRadial(startX, startY, x, y)) { fsBootArcArmed = false; return; }   // e' uno swipe
    float f;
    if (!fuelArcFracAt(x, y, f)) return;
    fsBootArcMoved = true;
    const float liters = roundf(f * (float)fuelTankL * 2.0f) / 2.0f;
    if (fabsf(liters - fsBootLiters) < 0.01f) return;
    fsBootLiters = liters;
    fsFullPressed = false;
    if (millis() - fsBootArcLastDrawMs >= 40UL) { fsBootArcLastDrawMs = millis(); drawFuelSetupScreen(); }
    return;
  }
  const unsigned long now = millis();
  if (touchId != fsTouchId) {
    fsTouchId = touchId; fsDragCol = -1; fsDragDecided = false;
    fsLastY = startY; fsLastMs = now; fsVel = 0.0f;
    if (fsFlingCol >= 0) { fsPos[fuelSetupPage][fsFlingCol] = (float)fsIndex(fuelSetupPage, fsFlingCol); fsFlingCol = -1; drawFuelSetupScreen(); }
    return;
  }
  if (!fsDragDecided) {
    const int dx = abs(x - startX), dy = abs(y - startY);
    if (dx < 8 && dy < 8) return;
    fsDragDecided = true;
    if (dy > dx && startY >= FS_AREA_Y0 - 20 && startY <= FS_AREA_Y1) fsDragCol = fsColAt(startX);
  }
  if (fsDragCol < 0) return;
  const int d = y - fsLastY;
  const unsigned long dt = max(1UL, now - fsLastMs);
  fsLastY = y; fsLastMs = now;
  if (d == 0) return;
  const float speed = fabsf((float)d) / (float)dt;
  const int n = FUEL_WHEELS[fuelSetupPage][fsDragCol].count;
  const float gain = n > 100 ? constrain(1.0f + (speed - 0.4f) * 4.0f, 1.0f, 6.0f) : 1.0f;
  const float steps = -(float)d / (float)FS_ROW * gain;
  fsPos[fuelSetupPage][fsDragCol] += steps;
  fsClamp(fuelSetupPage, fsDragCol);
  fsVel = 0.6f * fsVel + 0.4f * (steps / (float)dt);
  if (now - fsLastDrawMs >= 30UL) drawFuelSetupScreen();
}

// SAVE di FUEL SETUP / SET LITERS (anche allo scadere dei 30 s di SET LITERS all'accensione)
static void fuelSetupSave() {
  if (fsBootMode) {   // SET LITERS: cambiano solo i litri a bordo (e "since FULL" con FULL)
    fuelLiters = constrain(fsBootLiters, 0.0f, (float)fuelTankL);
    if (fsFullPressed) fuelSinceFullMs = 0;
    if (fuelDemoActive) fuelDemoLiters = fuelLiters;
    fuelReserveAlerted = fuelLiters <= fuelReserveLiters();
    saveFuelLiters();
    bleQueueFuel(fsFullPressed);   // slave: al master, che lo applica e lo rinvia
    AVEA_LOGF("FUEL: SET LITERS all'accensione: %.1f l\n", fuelLiters);
    fuelSetupExit();
    playTone(1500, 120);
    return;
  }
  fsFlingCol = -1;
  for (int pp = 0; pp < 2; ++pp) for (int c = 0; c < 3; ++c) if (FUEL_WHEELS[pp][c].label) fsPos[pp][c] = (float)fsIndex(pp, c);
  fuelTankL = (int)lroundf(fsValue(1, 0));
  // LITERS piu' alti del serbatoio: il serbatoio si allarga ai litri impostati (prima i litri venivano tagliati)
  if (fsValue(0, 0) > (float)fuelTankL) {
    fuelTankL = (int)ceilf(fsValue(0, 0));
    fsSet(1, 0, (float)fuelTankL);
  }
  fuelGroundL = fsValue(1, 1);
  fuelRateLph = fsValue(0, 1);
  fuelReserveMin = (int)lroundf(fsValue(0, 2));
  fuelLiters = constrain(fsValue(0, 0), 0.0f, (float)fuelTankL);
  if (fsFullPressed) fuelSinceFullMs = 0;
  if (fuelDemoActive) fuelDemoLiters = fuelLiters;   // in demo la pagina mostra subito i litri salvati
  fuelReserveAlerted = fuelLiters <= fuelReserveLiters();   // con meno della riserva gia' a terra: niente avviso al decollo
  saveFuelSettings();
  bleQueueFuel(fsFullPressed);   // slave: manda i valori al master, che li applica e li rinvia
  AVEA_LOGF("FUEL: %.1f l, %.1f l/h, riserva %d min, serbatoio %d l, terra %.1f l\n",
                fuelLiters, fuelRateLph, fuelReserveMin, fuelTankL, fuelGroundL);
  fuelSetupExit();
  playTone(1500, 120);
}

void fuelSetupTouchRelease(bool tapValid, int diffX, int diffY, int startX, int startY) {
  if (fsBootMode) {   // SET LITERS: arco, FULL, SAVE; swipe = salta senza salvare
    const bool armed = fsBootArcArmed, moved = fsBootArcMoved;
    fsBootArcArmed = false; fsBootArcMoved = false; fsBootArcTouchId = 0;
    if (armed && moved) { drawFuelSetupScreen(); playTone(1200, 50); return; }
    if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) { fuelSetupExit(); playTone(1500, 80); return; }
    if (!tapValid) return;
    if (startX >= 64 && startX < 180 && startY >= 258 && startY <= 312) {          // FULL
      fsBootLiters = (float)fuelTankL; fsFullPressed = true;
      drawFuelSetupScreen(); playTone(1200, 50);
    } else if (startX >= 180 && startX <= 296 && startY >= 258 && startY <= 312) { // SAVE
      fuelSetupSave();
    }
    return;
  }
  const unsigned long now = millis();
  const int p = fuelSetupPage;
  if (fsDragCol >= 0) {
    const int c = fsDragCol;
    fsDragCol = -1; fsDragDecided = false; fsTouchId = 0;
    if (now - fsLastMs < 80UL && fabsf(fsVel) > 0.004f && FUEL_WHEELS[p][c].count > 30) {
      fsFlingCol = c; fsFlingVel = fsVel; fsFlingLastMs = now;
    } else {
      fsPos[p][c] = (float)fsIndex(p, c);
      drawFuelSetupScreen();
    }
    return;
  }
  fsDragDecided = false; fsTouchId = 0;
  if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {   // esce senza salvare
    fsFlingCol = -1;
    fuelSetupExit();
    playTone(1500, 80);
    return;
  }
  if (!tapValid) return;
  if (startX >= 110 && startX <= 250 && startY >= 40 && startY <= 98) {   // SETUP 1/2 <-> 2/2
    fsFlingCol = -1;
    fuelSetupPage ^= 1;
    drawFuelSetupScreen();
    playTone(1500, 80);
    return;
  }
  const bool onSave = (p == 0) ? (startX >= 176 && startX <= 296 && startY >= 246 && startY <= 300)
                               : (startX >= 120 && startX <= 240 && startY >= 246 && startY <= 300);
  if (p == 0 && startX >= 64 && startX <= 176 && startY >= 246 && startY <= 300) {   // FULL
    fsSet(0, 0, fsValue(1, 0));
    fsFullPressed = true;
    drawFuelSetupScreen();
    playTone(1200, 50);
  } else if (onSave) {
    fuelSetupSave();
  } else if (startY >= FS_AREA_Y0 && startY <= FS_AREA_Y1 && (startY < FS_BAND_Y0 || startY > FS_BAND_Y1)) {
    const int c = fsColAt(startX);
    fsFlingCol = -1;
    fsPos[p][c] = (float)(fsIndex(p, c) + (startY < FS_BAND_Y0 ? 1 : -1));
    fsClamp(p, c);
    drawFuelSetupScreen();
    playTone(1200, 30);
  }
}

// Stima del carburante e avviso riserva: chiamata a ogni giro del loop.
void updateFuel() {
  const unsigned long now = millis();
  static unsigned long lastMs = 0;
  static bool lastRunning = false;
  static unsigned long lastSaveMs = 0;
  const unsigned long dt = lastMs ? now - lastMs : 0;
  lastMs = now;

  // Demo: carburante simulato, i valori veri restano fermi
  if (demoMode && !fuelDemoActive) { fuelDemoActive = true; fuelDemoLiters = fuelReserveLiters() + 2.0f; fuelReserveAlerted = false; }
  if (!demoMode && fuelDemoActive) {
    fuelDemoActive = false;
    fuelReserveAlerted = fuelLiters <= fuelReserveLiters();
    if (currentScreen == FUEL_RESERVE_SCREEN) { currentScreen = fuelAlertPrevious; redrawCurrentScreen(); }
  }

  const bool running = automaticFlightRunning;
  static float takeoffLiters = 0.0f;
  if (running && !lastRunning) takeoffLiters = fuelShownLiters();   // litri prima del consumo a terra
  if (running && !lastRunning) {   // decollo: consumo fisso a terra
    if (fuelDemoActive) fuelDemoLiters = max(0.0f, fuelDemoLiters - fuelGroundL);
    else fuelLiters = max(0.0f, fuelLiters - fuelGroundL);
  }
  // Il loop ora gira ogni 1-2 ms: i millisecondi si sommano e i litri si scalano una volta al secondo,
  // altrimenti sottrazioni cosi' piccole si perderebbero negli arrotondamenti del float.
  static unsigned long fuelPendingMs = 0;
  if (running && dt > 0 && dt < 5000UL) fuelPendingMs += dt;
  if (!running) fuelPendingMs = 0;
  if (fuelPendingMs >= 1000UL) {
    const float used = fuelRateLph * (float)fuelPendingMs / 3600000.0f;
    if (fuelDemoActive) fuelDemoLiters = max(0.0f, fuelDemoLiters - used * 10.0f);
    else { fuelLiters = max(0.0f, fuelLiters - used); fuelSinceFullMs += fuelPendingMs; }
    fuelPendingMs = 0;
  }
  // FUEL ESTIMATE del volo per FLIGHT LOG e log salvati: litri consumati dal decollo (a terra compresi).
  // Sullo slave, a fine volo, il valore arriva poi con il log del master.
  if (running) flightFuelUsedL = max(0.0f, takeoffLiters - fuelShownLiters());
  if (!fuelDemoActive && ((running && now - lastSaveMs >= 60000UL) || (!running && lastRunning))) {
    lastSaveMs = now;
    saveFuelLiters();   // ogni minuto in volo e all'atterraggio
  }
  lastRunning = running;

  // Avviso RESERVE (una volta per ogni discesa sotto la riserva, solo in volo)
  const bool inReserve = fuelShownLiters() <= fuelReserveLiters() + 0.01f;
  if (!inReserve) fuelReserveAlerted = false;
  // L'avviso compare anche se la pagina FUEL ESTIMATE e' nascosta in PAGES (come tutti gli altri avvisi)
  if (inReserve && !fuelReserveAlerted && running && !startupSequenceActive && !alertsDeferred() &&
      currentScreen != ICT_ALERT_SCREEN && currentScreen != FLIGHT_END_SCREEN &&
      currentScreen != RISK_WARNING_SCREEN && currentScreen != SUNSET_SCREEN &&
      currentScreen != FUEL_SETUP_SCREEN && currentScreen != FUEL_RESERVE_SCREEN &&
      currentScreen != AIRSPACE_ALERT_SCREEN && currentScreen != DENSITY_ALT_SCREEN) {
    fuelReserveAlerted = true;
    fuelAlertPrevious = currentScreen;
    currentScreen = FUEL_RESERVE_SCREEN;
    drawFuelScreen();
    playTone(900, 150); playTone(1500, 150); playTone(900, 150);
  }

  // Ridisegno: inerzia dei rulli del setup, pagina FUEL una volta al secondo
  if (currentScreen == FUEL_SETUP_SCREEN && fsFlingCol >= 0) {
    const float fdt = (float)(now - fsFlingLastMs);
    if (fdt >= 1.0f) {
      fsFlingLastMs = now;
      fsPos[fuelSetupPage][fsFlingCol] += fsFlingVel * fdt;
      fsFlingVel *= expf(-fdt / 300.0f);
      const int n = FUEL_WHEELS[fuelSetupPage][fsFlingCol].count;
      if (fsPos[fuelSetupPage][fsFlingCol] <= 0.0f || fsPos[fuelSetupPage][fsFlingCol] >= n - 1) fsFlingVel = 0.0f;
      fsClamp(fuelSetupPage, fsFlingCol);
      if (fabsf(fsFlingVel) < 0.003f) {
        fsPos[fuelSetupPage][fsFlingCol] = (float)fsIndex(fuelSetupPage, fsFlingCol);
        fsFlingCol = -1;
        drawFuelSetupScreen();
      } else if (now - fsLastDrawMs >= 30UL) drawFuelSetupScreen();
    }
  } else if (currentScreen != FUEL_SETUP_SCREEN) {
    fsFlingCol = -1;
  }
  // In riserva ogni mezzo secondo (titolo lampeggiante), altrimenti ogni secondo
  if ((currentScreen == FUEL_SCREEN || currentScreen == FUEL_RESERVE_SCREEN) &&
      now - fuelLastDrawMs >= (inReserve ? 500UL : 1000UL)) {
    drawFuelScreen();
  }
}

// Volo reale: se al decollo si e' su una pagina nascosta (GPS INFO, WIFI LOG, SETUP e sue pagine, ICT)
// o nelle impostazioni carburante, si passa alla pagina successiva del giro. Chiamata a ogni giro del loop.
void applyFlightPageLock() {
  if (!realFlightActive()) return;
  const bool locked = currentScreen == GPS_INFO_SCREEN || currentScreen == WIFI_LOGS_SCREEN ||
                      currentScreen == AV_SETUP_SCREEN || currentScreen == DUAL_MODE_SCREEN ||
                      currentScreen == ICT_SCREEN || currentScreen == FUEL_SETUP_SCREEN ||
                      currentScreen == FIRMWARE_SCREEN || currentScreen == PAGE_SELECT_SCREEN ||
                      currentScreen == ALT_OFFSET_SCREEN || currentScreen == WIFI_LOGIN_SCREEN;
  if (!locked) return;
  if (currentScreen == FUEL_SETUP_SCREEN && fsBootMode) {   // SET LITERS di accensione: pagina salvata, senza salvare
    fsBootMode = false;
    AVEA_LOGLN("Volo reale: SET LITERS chiusa");
    bootWizardFinish();
    return;
  }
  if (currentScreen == FUEL_SETUP_SCREEN) {   // esce senza salvare
    currentScreen = FUEL_SCREEN;
  } else {
    int idx = currentScreenIndex;
    for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == currentScreen) { idx = i; break; }
    if (currentScreen == DUAL_MODE_SCREEN) {
      for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == AV_SETUP_SCREEN) { idx = i; break; }
    }
    if (currentScreen == PAGE_SELECT_SCREEN || currentScreen == ALT_OFFSET_SCREEN) {
      for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == FIRMWARE_SCREEN) { idx = i; break; }
    }
    currentScreenIndex = nextEnabledScreenIndex(idx, 1);
    currentScreen = screenOrder[currentScreenIndex];
  }
  AVEA_LOGLN("Volo reale: pagina di impostazione chiusa");
  redrawCurrentScreen();
}

// ---- Regolazione dei litri trascinando il dito sull'arco della pagina FUEL ESTIMATE ----
// Solo a terra (bloccata in volo reale per non spostarla per sbaglio). Passi da 0.5 l.
static bool fuelArcDragArmed = false, fuelArcDragMoved = false;
static unsigned long fuelArcTouchId = 0, fuelArcLastDrawMs = 0;
// Frazione 0..1 dell'arco sotto il dito; false se il dito non e' sull'arco.
static bool fuelArcFracAt(int x, int y, float &frac) {
  const float dx = (float)x - 180.0f, dy = (float)y - 180.0f;
  const float r = sqrtf(dx * dx + dy * dy);
  if (r < (float)(FUEL_ARC_R - FUEL_ARC_HALF - 22)) return false;   // un po' di margine verso l'interno
  float a = atan2f(dy, dx) * RAD_TO_DEG;                           // 0 = ore 3, senso orario (y in basso)
  if (a < 0.0f) a += 360.0f;
  if (a < 90.0f) a += 360.0f;                                       // l'arco va da 135 a 405 gradi
  if (a < 120.0f || a > 420.0f) return false;                       // parte bassa senza arco (15 gradi di tolleranza)
  frac = constrain((a - 135.0f) / 270.0f, 0.0f, 1.0f);
  return true;
}
void fuelArcTouchMove(int startX, int startY, int x, int y, unsigned long touchId) {
  if (currentScreen != FUEL_SCREEN) return;
  if (touchId != fuelArcTouchId) {
    fuelArcTouchId = touchId;
    fuelArcDragMoved = false;
    float f0;
    fuelArcDragArmed = !realFlightActive() && fuelArcFracAt(startX, startY, f0);   // in volo reale: bloccato
    return;
  }
  if (!fuelArcDragArmed) return;
  if (!fuelArcDragMoved && abs(x - startX) < 6 && abs(y - startY) < 6) return;
  if (!fuelArcDragMoved && arcDragIsRadial(startX, startY, x, y)) { fuelArcDragArmed = false; return; }   // e' uno swipe
  float f;
  if (!fuelArcFracAt(x, y, f)) return;
  fuelArcDragMoved = true;
  const float liters = roundf(f * (float)fuelTankL * 2.0f) / 2.0f;
  if (fabsf(liters - fuelLiters) < 0.01f) return;
  fuelLiters = liters;
  if (fuelDemoActive) fuelDemoLiters = liters;
  if (millis() - fuelArcLastDrawMs >= 40UL) { fuelArcLastDrawMs = millis(); drawFuelScreen(); }
}
// Al rilascio; true = gestito (il dito era sull'arco: niente cambio pagina, niente apertura del setup).
bool fuelArcTouchRelease() {
  if (currentScreen != FUEL_SCREEN) return false;
  const bool armed = fuelArcDragArmed, moved = fuelArcDragMoved;
  fuelArcDragArmed = false; fuelArcDragMoved = false; fuelArcTouchId = 0;
  if (!armed) return false;
  if (moved) {
    fuelReserveAlerted = fuelLiters <= fuelReserveLiters();
    saveFuelLiters();
    bleQueueFuel(false);   // slave: al master, che lo rinvia; il master lo invia da solo ogni secondo
    AVEA_LOGF("FUEL: litri regolati sull'arco: %.1f l\n", fuelLiters);
    playTone(1200, 50);
  }
  drawFuelScreen();
  return true;
}

// Avviso RESERVE: tocco o swipe = torna alla pagina di prima.
void dismissFuelReserveScreen() {
  currentScreen = fuelAlertPrevious;
  redrawCurrentScreen();
}

void drawLubberLineScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize(1.1f * THEME_TEXT_SCALE, 1.1f * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("LUBBER LINE", 180, 42);

  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1.0f * THEME_TEXT_SCALE, 1.0f * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("GREEN WIDTH (km)", 180, 80);
  lcd.drawString("YELLOW MARGIN (+km)", 180, 176);

  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize(1.0f * THEME_TEXT_SCALE, 1.0f * THEME_TEXT_SCALE);
  const char* widthLabels[4] = { "1", "2", "3", "5" };
  const char* yellowLabels[3] = { "+1", "+2", "+3" };
  for (int row = 0; row < 2; ++row) {
    const int count = row == 0 ? 4 : 3;
    const int boxW = row == 0 ? 60 : 80;
    const int stepX = row == 0 ? 68 : 90;
    const int x0 = row == 0 ? 48 : 50;
    const int boxY = row == 0 ? 96 : 192;
    const int boxH = 44;
    const int selectedIdx = row == 0 ? lubberWidthIdx : lubberYellowIdx;
    for (int i = 0; i < count; ++i) {
      const int boxX = x0 + i * stepX;
      const bool selected = (i == selectedIdx);
      const uint16_t accent = selected ? themeColor(TFT_GREEN) : themeColor(TFT_DARKGREY);
      if (selected) lcd.fillRoundRect(boxX, boxY, boxW, boxH, 8, accent);
      else lcd.drawRoundRect(boxX, boxY, boxW, boxH, 8, accent);
      if (selected) {
        const uint16_t selectedText = displayTheme == THEME_COLOR ? TFT_BLACK : themeBg();
        lcd.setTextColor(selectedText, accent);
      } else {
        setThemeTextColor(TFT_WHITE, TFT_BLACK);
      }
      lcd.drawString(row == 0 ? widthLabels[i] : yellowLabels[i], boxX + boxW / 2, boxY + boxH / 2);
    }
  }
}

// ---- Archi laterali della SENSOR Dashboard: scala -20..+40 C dal basso in alto, zone colorate fisse
// (blu <0, azzurro 0-10, verde 10-25, arancio 25-35, rosso >35), tacche e numeri 0 e 20, indice a triangolo.
// Arco sinistro = temperatura mostrata grande (EXT con la sonda, altrimenti quella corretta); arco destro = INT.
static const float OAT_ARC_RIN = 162.0f;
static float oatArcAngle(float t) { const float f = constrain((t + 20.0f) / 60.0f, 0.0f, 1.0f); return -50.0f + 100.0f * f; }
static void oatArcPt(bool left, float r, float ang, float &x, float &y) {
  const float a = ang * DEG_TO_RAD;
  x = 179.5f + (left ? -1.0f : 1.0f) * r * cosf(a); y = 179.5f - r * sinf(a);
}
// Scala temperatura -20..+40 C, un colore ogni 5 gradi, sfumata (A).
static uint16_t oatGradColor(float t) {
  static const uint8_t c[13][3] = {{74, 20, 140}, {26, 35, 126}, {13, 71, 161}, {25, 118, 210}, {0, 172, 193},
                                   {38, 166, 154}, {102, 187, 106}, {212, 225, 87}, {255, 238, 88}, {255, 167, 38},
                                   {245, 124, 0}, {229, 57, 53}, {142, 0, 0}};
  t = constrain(t, -20.0f, 40.0f);
  int i = (int)((t + 20.0f) / 5.0f);
  if (i > 11) i = 11;
  const float f = (t - (-20.0f + 5.0f * i)) / 5.0f;
  uint8_t rgb[3];
  for (int k = 0; k < 3; ++k) rgb[k] = (uint8_t)lroundf(c[i][k] + (c[i + 1][k] - c[i][k]) * f);
  return lgfx::color565(rgb[0], rgb[1], rgb[2]);
}
static void drawOatArc(bool left) {
  const int STEPS = 60;   // un tratto ogni grado: giunte interne nette, bordi morbidi solo alle estremita'
  for (int i = 0; i < STEPS; ++i) {
    const float t0 = -20.0f + i, t1 = t0 + 1.0f;
    fillArcAAPart(lcd, 179.5f, 179.5f, OAT_ARC_RIN, ARC_R_OUT, oatArcAngle(t0), oatArcAngle(t1), left,
                  oatGradColor(t0 + 0.5f), themeBg(), i == 0, i == STEPS - 1);
  }
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1.0f, 1.0f);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(themeColor(TFT_LIGHTGREY), themeBg());
  const float marks[2] = {0.0f, 20.0f};
  for (int i = 0; i < 2; ++i) {
    const float ang = oatArcAngle(marks[i]);
    float x0, y0, x1, y1;
    oatArcPt(left, OAT_ARC_RIN - 1.0f, ang, x0, y0); oatArcPt(left, ARC_R_OUT + 1.0f, ang, x1, y1);
    lcd.drawWideLine(x0, y0, x1, y1, 0.9f, themeBg());   // tacca nera attraverso l'arco
    float lx, ly; oatArcPt(left, 132.0f, ang, lx, ly);
    lcd.drawString(i == 0 ? "0" : "20", (int)lroundf(lx), (int)lroundf(ly));
  }
}
static void drawOatArcMarker(bool left, float t, uint16_t color) {   // triangolo dall'interno che punta all'arco
  const float ang = oatArcAngle(t);
  float ax, ay, bx, by, cx, cy;
  oatArcPt(left, OAT_ARC_RIN - 2.0f, ang, ax, ay);
  oatArcPt(left, OAT_ARC_RIN - 16.0f, ang - 4.5f, bx, by);
  oatArcPt(left, OAT_ARC_RIN - 16.0f, ang + 4.5f, cx, cy);
  lcd.fillTriangle((int)lroundf(ax), (int)lroundf(ay), (int)lroundf(bx), (int)lroundf(by), (int)lroundf(cx), (int)lroundf(cy), color);
}
static float oatMarkL = -999.0f, oatMarkR = -999.0f;   // ultimo valore indicato (-999 = nessuno)
static void updateOatArcMarker(bool left, float t) {
  float &last = left ? oatMarkL : oatMarkR;
  if (last > -500.0f && fabsf(t - last) < 0.25f) return;
  if (last > -500.0f) drawOatArcMarker(left, last, themeBg());
  drawOatArcMarker(left, t, themeColor(TFT_WHITE));
  last = t;
}

void drawMeteoScreen() {
  lcd.fillScreen(themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.25f) * THEME_TEXT_SCALE, (1.25f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("SENSOR", 180, 42, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  drawUiVlw("Dashboard", 180, 72, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.drawRect(85, 275, 190, 45, themeColor(TFT_YELLOW));
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  drawUiVlw("AIR DATA", 180, 298, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.82f);
  oatMarkL = oatMarkR = -999.0f;
  drawOatArc(true);                       // arco sinistro sempre
  if (oatValid) {                         // con la sonda: arco destro INT e titoli EXT / INT
    drawOatArc(false);
    drawUiVlw("EXT", 120, 108, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.8f);
    drawUiVlw("INT", 240, 108, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.8f);
  }
  lastPrintedMeteoTemp = -99999;
  lastPrintedMeteoHumidity = -99999;
  updateMeteoData();
}

// ========== SONDA ESTERNA DS18B20 (OAT) su GPIO13 (connettore J7, pin 18) ==========
// 1-Wire a 3 fili (3V3, GND, dati con resistenza da 4,7 kOhm verso 3V3). Lettura ogni 2 s, non bloccante:
// si avvia la conversione e si legge 800 ms dopo. Con la sonda collegata, tutte le stime meteo usano la
// temperatura esterna e l'umidita' ricalcolata dal punto di rugiada del BME280 (stesso vapore). Senza
// sonda (3 letture fallite di fila) si torna da soli alla lettura interna con la correzione di METEO OFFSET.
#define OAT_PIN 13
volatile bool oatValid = false;
volatile float oatTempC = NAN;
volatile bool oatFromRemote = false;   // slave BLE: valore della sonda del master
static uint8_t oatFails = 0;
static portMUX_TYPE owMux = portMUX_INITIALIZER_UNLOCKED;
static inline void owLow() { gpio_set_level((gpio_num_t)OAT_PIN, 0); }
static inline void owRel() { gpio_set_level((gpio_num_t)OAT_PIN, 1); }
static inline int owIn() { return gpio_get_level((gpio_num_t)OAT_PIN); }
static void oatInit() {
  gpio_reset_pin((gpio_num_t)OAT_PIN);
  gpio_set_direction((gpio_num_t)OAT_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_pull_mode((gpio_num_t)OAT_PIN, GPIO_PULLUP_ONLY);   // senza sonda la linea resta alta: nessuna presenza
  owRel();
}
static bool owReset() {
  owLow();
  delayMicroseconds(480);
  portENTER_CRITICAL(&owMux);
  owRel();
  esp_rom_delay_us(70);
  const bool presence = owIn() == 0;
  portEXIT_CRITICAL(&owMux);
  delayMicroseconds(410);
  return presence && owIn() == 1;   // la linea deve tornare alta (niente cortocircuiti)
}
static void owWriteBit(int b) {
  portENTER_CRITICAL(&owMux);
  owLow();
  esp_rom_delay_us(b ? 6 : 60);
  owRel();
  esp_rom_delay_us(b ? 64 : 10);
  portEXIT_CRITICAL(&owMux);
}
static int owReadBit() {
  portENTER_CRITICAL(&owMux);
  owLow();
  esp_rom_delay_us(3);
  owRel();
  esp_rom_delay_us(10);
  const int r = owIn();
  portEXIT_CRITICAL(&owMux);
  esp_rom_delay_us(53);
  return r;
}
static void owWrite(uint8_t v) { for (int i = 0; i < 8; ++i) owWriteBit((v >> i) & 1); }
static uint8_t owRead() { uint8_t v = 0; for (int i = 0; i < 8; ++i) if (owReadBit()) v |= (uint8_t)(1 << i); return v; }
static uint8_t owCrc8(const uint8_t *d, int n) {
  uint8_t crc = 0;
  for (int i = 0; i < n; ++i) {
    uint8_t in = d[i];
    for (int b = 0; b < 8; ++b) { const uint8_t mix = (crc ^ in) & 1; crc >>= 1; if (mix) crc ^= 0x8C; in >>= 1; }
  }
  return crc;
}
static void oatFail() {
  if (oatFails < 255) ++oatFails;
  if (oatFails >= 3 && !oatFromRemote) { oatValid = false; oatTempC = NAN; }
}
// Chiamata a ogni giro del loop.
void updateOatProbe() {
  static unsigned long lastStartMs = 0;
  static bool converting = false;
  const unsigned long now = millis();
  if (!converting) {
    if (lastStartMs != 0 && now - lastStartMs < 2000UL) return;
    lastStartMs = now;
    if (!owReset()) { oatFail(); return; }
    owWrite(0xCC);   // SKIP ROM (una sola sonda)
    owWrite(0x44);   // CONVERT T
    converting = true;
    return;
  }
  if (now - lastStartMs < 800UL) return;   // 12 bit: 750 ms
  converting = false;
  if (!owReset()) { oatFail(); return; }
  owWrite(0xCC);
  owWrite(0xBE);     // READ SCRATCHPAD
  uint8_t d[9];
  for (int i = 0; i < 9; ++i) d[i] = owRead();
  bool allSame = true;
  for (int i = 1; i < 9; ++i) if (d[i] != d[0]) allSame = false;
  if (allSame || owCrc8(d, 8) != d[8]) { oatFail(); return; }
  const float t = (float)(int16_t)((uint16_t)d[1] << 8 | d[0]) / 16.0f;
  if (t < -55.0f || t > 85.0f || t == 85.0f) { oatFail(); return; }   // 85.0 = valore di accensione, non una misura
  oatTempC = t;
  oatValid = true;
  oatFromRemote = false;   // la sonda propria vale sempre piu' di quella del master
  oatFails = 0;
}

// Il display, montato vicino al sensore meteo, lo scalda: la temperatura
// letta risulta piu' alta di circa 2-3 gradi rispetto all'aria esterna reale,
// e di conseguenza l'umidita' relativa calcolata risulta piu' bassa di circa
// il 7%. Questi offset compensano l'effetto per tutte le stime meteo (dew
// point, cloud base, density altitude, rischio ghiaccio carburatore e nebbia)
// cosi' da non sottostimare i rischi per via dell'autoriscaldamento.
const float OAT_SENSOR_HEATING_OFFSET_C = 0.0f;                        // offset temperatura
const float OAT_SENSOR_HUMIDITY_COMPENSATION_PCT = 0.0f;                // offset umidità
// METEO OFFSET (PAGES 2/2): correzione della temperatura del sensore, in gradi (es. -2.0 se legge 2 gradi in piu').
// L'umidita' relativa viene ricalcolata per la nuova temperatura a parita' di vapore (stesso punto di rugiada):
// se il sensore e' scaldato, l'aria vera e' piu' fredda e la sua umidita' relativa e' piu' alta.
static float satVaporHpa(float t) { return 6.112f * expf(17.62f * t / (243.12f + t)); }
// Che cosa fa SAVE in METEO OFFSET: 1 = taratura a freddo (primi 5 minuti dall'accensione: offset e
// riferimento del chip), 2 = taratura a caldo (strumento scaldato di almeno 2 gradi rispetto al freddo:
// si calcola la compensazione), 0 = solo offset (chip non leggibile o non ancora tarato a freddo).
int meteoCalibMode(float rawTemp) {
  if (!isfinite(chipTempC) || !isfinite(rawTemp)) return 0;
  if (millis() < 5UL * 60000UL) return 1;
  if (isfinite(meteoHeatD0) && (chipTempC - rawTemp) - meteoHeatD0 > 2.0f) return 2;
  return 0;
}
// Lettura del termometro interno dell'ESP32 ogni 2 secondi, con un filtro (cambia lentamente).
void updateChipTemperature() {
  static unsigned long lastMs = 0;
  if (lastMs != 0 && millis() - lastMs < 2000UL) return;
  lastMs = millis();
  const float t = temperatureRead();
  if (!isfinite(t) || t < -40.0f || t > 125.0f) return;
  chipTempC = isfinite(chipTempC) ? chipTempC + (t - chipTempC) * 0.2f : t;
}
// Gradi da togliere per il calore interno (0 se la compensazione non e' tarata o il chip non e' leggibile).
float meteoHeatCorrection(float rawTemp) {
  const float chip = chipTempC;
  if (meteoHeatK <= 0.0f || !isfinite(meteoHeatD0) || !isfinite(chip) || !isfinite(rawTemp)) return 0.0f;
  return meteoHeatK * max(0.0f, (chip - rawTemp) - meteoHeatD0);
}
// La stima del sensore interno: sensore + offset, meno il peso del calore interno calcolato
// col sensore di temperatura del processore. E' il numero che lo strumento usa per tutte le
// stime meteo quando la sonda esterna non c'e'. Sta in una funzione sola perche' la usano sia
// la compensazione qui sotto sia la pagina SENSOR: scritta due volte, prima o poi divergono.
float meteoInternalCorrectedC(float rawTemp) {
  if (!isfinite(rawTemp)) return NAN;
  return rawTemp - OAT_SENSOR_HEATING_OFFSET_C + meteoTempOffsetC - meteoHeatCorrection(rawTemp);
}
bool meteoForceInternal = false;   // true = ignora la sonda esterna (anteprima di METEO OFFSET)
void compensateOutsideAirReading(float rawTemp, float rawHumidity, float &compTemp, float &compHumidity) {
  const float oat = oatTempC;
  if (oatValid && !meteoForceInternal && isfinite(oat) && isfinite(rawTemp) && isfinite(rawHumidity) &&
      rawTemp > -60.0f && rawTemp < 80.0f) {
    // Sonda esterna: temperatura vera dell'aria; l'umidita' si ricalcola a parita' di vapore (stesso punto di rugiada).
    compTemp = oat;
    compHumidity = constrain(rawHumidity * satVaporHpa(rawTemp) / satVaporHpa(oat), 0.0f, 100.0f);
    return;
  }
  compTemp = meteoInternalCorrectedC(rawTemp);
  float h = rawHumidity + OAT_SENSOR_HUMIDITY_COMPENSATION_PCT;
  if (compTemp != rawTemp && isfinite(rawTemp) && isfinite(h) && rawTemp > -60.0f && rawTemp < 80.0f && compTemp > -60.0f)
    h = h * satVaporHpa(rawTemp) / satVaporHpa(compTemp);
  compHumidity = constrain(h, 0.0f, 100.0f);
}

float calculateDewPointSafe(float temp, float hum) {
  if (!isfinite(temp) || !isfinite(hum) || hum <= 0.0f || hum > 100.0f || temp <= -60.0f || temp >= 60.0f) return NAN;
  const float a = 17.27f;
  const float b = 237.7f;
  float gamma = (a * temp) / (b + temp) + log(hum / 100.0f);
  return (b * gamma) / (a - gamma);
}

float calculateCloudBase(float temp, float dewPoint) {
  if (!isfinite(temp) || !isfinite(dewPoint) || dewPoint > temp) return NAN;
  return max(0.0f, (temp - dewPoint) * 125.0f);
}

float calculateDensityAltitude(float tempC, float qnh_hPa, float indicatedAlt_m) {
  if (!isfinite(tempC) || !isfinite(qnh_hPa) || !isfinite(indicatedAlt_m) ||
      qnh_hPa < QNH_MIN_HPA || qnh_hPa > QNH_MAX_HPA) return NAN;
  float pressureAltitudeFeet = (indicatedAlt_m * 3.28084f) + (27.0f * (1013.25f - qnh_hPa));
  float isaTempC = 15.0f - (2.0f * (pressureAltitudeFeet / 1000.0f));
  return pressureAltitudeFeet + (120.0f * (tempC - isaTempC));
}

int calculateCarbIceRisk(float temp, float humidity) {
  if (!isfinite(temp) || !isfinite(humidity)) return 0;
  if (temp < -10.0f || temp > 38.0f || humidity < 50.0f) return 0;
  if (temp <= 22.0f && humidity >= 80.0f) return 2;   // condizioni ambientali molto favorevoli; non e' una diagnosi del motore
  if (temp <= 30.0f && humidity >= 50.0f) return 1;   // condizioni ambientali favorevoli; la potenza del motore non e' considerata
  return 0;
}

// Foschia possibile (solo informativa, NON fa scattare avvisi): spread T - Td tra 4 e 8 C.
// Tipico della foschia padana; la visibilita' vera dipende anche da polveri e inversione (vedi METAR).
bool calculateHazePossible(float temp, float dewPoint) {
  if (!isfinite(temp) || !isfinite(dewPoint)) return false;
  const float spread = temp - dewPoint;
  return spread > 4.0f && spread <= 8.0f;
}

int calculateFogRisk(float temp, float dewPoint) {
  if (!isfinite(temp) || !isfinite(dewPoint)) return 0;
  float spread = temp - dewPoint;
  if (spread <= 2.0f) return 2;
  if (spread <= 4.0f) return 1;
  return 0;
}

void drawInvalidSensorMessage(const char *message) {
  lcd.fillRect(10, 105, 340, 150, themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.setTextColor(themeColor(TFT_RED), themeBg());
  lcd.drawString(message, 180, 175);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.drawString("CHECK SENSOR / DATA", 180, 220);
}

// [FIX B] updateMainData: pulizia del messaggio d'errore quando il dato torna valido.
void updateMainData() {
  if (currentScreen != MAIN_ALT_SCREEN) return;
  updateAltmQnhBox(false);   // ridisegna solo se il QNH e' cambiato

  float localAlt = 0.0f;
  bool altitudeValid = false;
  if (sensorDataMutex == nullptr ||
      xSemaphoreTake(sensorDataMutex, (TickType_t)10) != pdTRUE) return;
  localAlt = sharedFilteredAlt;
  altitudeValid = sharedAltitudeValid && isfinite(localAlt);
  const float localVarioMain = sharedVarioKalman;
  const bool varioValidMain = sharedVarioValid && isfinite(localVarioMain);
  xSemaphoreGive(sensorDataMutex);
  if (!altitudeValid) {
    if (!mainAltitudeErrorDisplayed) {
      drawInvalidSensorMessage("ALT INVALID");
      mainAltitudeErrorDisplayed = true;
      updateMainGsGauge(true);
    } else {
      updateMainGsGauge(false);
    }
    return;
  }
  if (mainAltitudeErrorDisplayed) {
    lcd.fillRect(10, 105, 340, 150, themeBg());
    lastPrintedAlt = -99999;
    lastPrintedUnit = "";
    mainAltitudeErrorDisplayed = false;
    // Il fillRect sopra puo' aver cancellato parte delle barre del variometro: ridisegno tutto.
    lastMainVarioTenth = 99999;
    lastMainVarioTextW = 0;
    lastMainVarioSign = 99;
    lastMainVarioLit = -1;
    drawAltVarioZeroLine();
    gsLastShownKmh = -2;
  }
  const float displayAltitude = altitudeUnitsIsMeters ? localAlt : localAlt * 3.28084f;
  int dispAlt = (int)round(displayAltitude);
  const char* unitStr = altitudeUnitsIsMeters ? "mt" : "ft";
  // Un po' di isteresi evita che il numero alterni rapidamente quando il
  // valore filtrato oscilla attorno alla soglia di arrotondamento (x,5).
  if (lastPrintedAlt != -99999 && strcmp(unitStr, lastPrintedUnit) == 0 &&
      fabsf(displayAltitude - (float)lastPrintedAlt) < 0.65f) {
    dispAlt = lastPrintedAlt;
  }
  if (dispAlt != lastPrintedAlt || strcmp(unitStr, lastPrintedUnit) != 0) {
    char altBuf[32];
    snprintf(altBuf, sizeof(altBuf), "%d", dispAlt);
    const bool unitChanged = strcmp(unitStr, lastPrintedUnit) != 0;
    const int altCenterX = 180;
    if (!drawMainAltitudeSprite(altBuf)) {
      // Ripiego senza sprite (memoria insufficiente): vecchio disegno diretto + ridisegno di cio' che cancella.
      setThemeTextColor(TFT_WHITE, TFT_BLACK);
      drawNumericVlw(altBuf, altCenterX, 165, MC_DATUM, TFT_WHITE, themeBg(), 1.14f, 6);
      gsLastShownKmh = -2;            // scala GS e lancetta ridisegnate a fine funzione
      lastMainVarioTenth = 99999;
      lastMainVarioTextW = 0;
      lastMainVarioSign = 99;
      lastMainVarioLit = -1;
      drawAltVarioZeroLine();
      lastPrintedUnit = "";           // forza anche il ridisegno dell'unita'
    }
    if (unitChanged || lastPrintedUnit[0] == '\0') {
      lcd.fillRect(140, 203, 80, 27, themeBg());
      lcd.setTextSize((1.3f) * THEME_TEXT_SCALE, (1.3f) * THEME_TEXT_SCALE);
      lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
      setThemeTextColor(TFT_CYAN, TFT_BLACK);
      lcd.setTextDatum(MC_DATUM);
      lcd.drawString(unitStr, altCenterX, ALT_UNIT_Y);   // unita' al centro, sotto il numero
      lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    }
    lastPrintedAlt = dispAlt;
    lastPrintedUnit = unitStr;
  }
  updateMainAltVario(varioValidMain, localVarioMain);
  updateMainGsGauge(false);
}

// [FIX B] updateMeteoData: pulizia del messaggio d'errore quando il dato torna valido.
void updateMeteoData() {
  if (currentScreen != METEO_SCREEN) return;
  static bool lastMeteoValid = true;

  float temperature = 0.0f;
  float humidity = 0.0f;
  bool valid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    temperature = meteo_temperature_c;
    humidity = meteo_humidity_percent;
    valid = meteoValid && isfinite(temperature) && isfinite(humidity);
    xSemaphoreGive(sensorDataMutex);
  }
  if (!valid) {
    drawInvalidSensorMessage("METEO INVALID");
    lastMeteoValid = false;
    return;
  }
  // Con la sonda esterna: grande la temperatura esterna (EXT) un po' piu' in alto, sotto piu' piccola quella
  // interna del BME280 senza correzione (INT). Senza sonda la pagina resta com'era, con la correzione.
  const bool extMode = oatValid;
  static int lastExtMode = -1;
  if (lastExtMode != (int)extMode) {   // sonda collegata o staccata: pagina ridisegnata da capo
    lastExtMode = (int)extMode;
    drawMeteoScreen();
    return;
  }
  const float rawInternal = temperature;
  // Mostra sulla schermata Meteo i valori corretti per l'autoriscaldamento
  // del sensore, con la stessa compensazione usata per il calcolo del rischio.
  float compensatedTemperature, compensatedHumidity;
  compensateOutsideAirReading(temperature, humidity,
                               compensatedTemperature, compensatedHumidity);
  temperature = compensatedTemperature;
  humidity = compensatedHumidity;
  static int lastDegreeX = -1;
  if (!lastMeteoValid) {   // dopo "METEO INVALID" la pagina si ridisegna da capo (archi compresi)
    lastMeteoValid = true;
    lastDegreeX = -1;
    drawMeteoScreen();
    return;
  }
  int tempValue = (int)round(temperature);
  int humidityValue = (int)round(humidity);
  if (lastPrintedMeteoTemp == -99999) lastDegreeX = -1;   // pagina appena ridisegnata: niente vecchio segno da cancellare
  if (extMode) {
    // EXT e INT affiancati, stessa grandezza; archi a zone: sinistro EXT, destro INT (senza correzione)
    static int lastIntShown = -99999;
    if (lastPrintedMeteoTemp == -99999) lastIntShown = -99999;
    const float numScale = 0.8f, ratio = numScale / 0.85f;
    const int numY = 165;
    const int degY = numY - (int)lroundf(30.0f * ratio);
    const int dRo = (int)lroundf(5.0f * ratio), dRi = (int)lroundf(4.0f * ratio);
    auto drawSide = [&](int value, int cx, int clearX, int clearW, uint16_t numColor, uint16_t degColor, int degGap) {
      lcd.fillRect(clearX, 124, clearW, 76, themeBg());
      char b[16];
      snprintf(b, sizeof(b), "%d", value);
      const int w = drawNumericVlw(b, cx, numY, MC_DATUM, numColor, themeBg(), numScale, 0);   // fondo gia' pulito dal riquadro
      const int dx = cx + w / 2 + degGap;
      lcd.drawCircle(dx, degY, dRo, degColor);
      lcd.drawCircle(dx, degY, dRi, degColor);
    };
    if (tempValue != lastPrintedMeteoTemp) {
      drawSide(tempValue, 118, 66, 112, TFT_WHITE, themeColor(TFT_YELLOW), 12);
      lastPrintedMeteoTemp = tempValue;
    }
    // INT e' la stima del sensore interno CORRETTA, la stessa che lo strumento usa quando
    // stacchi la sonda: offset piu' peso del calore col sensore del processore. Affiancata a
    // EXT serve a verificare la taratura: se i due numeri stanno vicini, la taratura tiene.
    const float intCorr = meteoInternalCorrectedC(rawInternal);
    const int intValue = (int)round(isfinite(intCorr) ? intCorr : rawInternal);
    if (intValue != lastIntShown) {
      // INT scritta come EXT: numero bianco e cerchietto dei gradi giallo. Il grigio chiaro
      // faceva sembrare la stima del sensore interno un dato di seconda scelta, mentre serve
      // proprio a essere confrontata con la sonda, e due numeri da confrontare si scrivono uguali.
      drawSide(intValue, 232, 182, 110, TFT_WHITE, themeColor(TFT_YELLOW), 8);
      lastIntShown = intValue;
    }
    updateOatArcMarker(true, temperature);
    updateOatArcMarker(false, isfinite(intCorr) ? intCorr : rawInternal);
  }
  if (!extMode && tempValue != lastPrintedMeteoTemp) {
    lcd.setTextDatum(MC_DATUM);
    setThemeTextColor(TFT_WHITE, TFT_BLACK);
    char tempBuf[16];
    snprintf(tempBuf, sizeof(tempBuf), "%d", tempValue);
    const int tempCenterX = 180;
    // Numero centrale ingrandito del 20% (0.85 -> 1.02), come vario/g-meter/
    // altimetro. tempScaleRatio serve a spostare il segno ° in proporzione.
    const float tempNumScale = 1.02f;
    const float tempScaleRatio = tempNumScale / 0.85f;
    // Geometria del segno ° (dipende solo dalla scala, non dalla larghezza del numero).
    const int degreeYOffset = (int)lroundf(30.0f * tempScaleRatio);
    const int degreeY = 155 - degreeYOffset;
    const int degreeROuter = (int)lroundf(5.0f * tempScaleRatio);
    const int degreeRInner = (int)lroundf(4.0f * tempScaleRatio);
    const int degreeClearHalf = degreeROuter + 2;  // riquadro stretto: solo il cerchietto
    // [FIX GHOST] cancella il vecchio ° PRIMA di scrivere il numero: se lo cancellassi
    // dopo, il riquadro coprirebbe parte delle cifre quando il numero si allarga (es. 9 -> 10).
    if (lastDegreeX >= 0) {
      lcd.fillRect(lastDegreeX - degreeClearHalf, degreeY - degreeClearHalf, degreeClearHalf * 2 + 1, degreeClearHalf * 2 + 1, themeBg());
    }
    // [FIX GHOST DIGIT] clearMaxChars=4 copre il caso peggiore "-99".
    int tempWidth = drawNumericVlw(tempBuf, tempCenterX, 155, MC_DATUM, TFT_WHITE, themeBg(), tempNumScale, 4);
    int degreeMargin = (int)lroundf(14.0f * tempScaleRatio);
    int degreeX = tempCenterX + (tempWidth / 2) + degreeMargin;
    // Il ° si disegna sopra al fondo senza riquadro di cancellazione, cosi' non tocca le cifre.
    lcd.drawCircle(degreeX, degreeY, degreeROuter, themeColor(TFT_YELLOW));
    lcd.drawCircle(degreeX, degreeY, degreeRInner, themeColor(TFT_YELLOW));
    lastDegreeX = degreeX;
    lastPrintedMeteoTemp = tempValue;
  }
  if (!extMode) updateOatArcMarker(true, temperature);   // senza sonda: solo l'arco sinistro, temperatura corretta

  if (humidityValue != lastPrintedMeteoHumidity) {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize(1.43f, 1.43f);  // +30% rispetto alla dimensione precedente 1.1
    setThemeTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString("Humidity", 135, 235);
    char humidityValueBuf[12];
    snprintf(humidityValueBuf, sizeof(humidityValueBuf), "%d", humidityValue);
    // [FIX GHOST DIGIT] clearMaxChars=3 copre il caso peggiore "100".
    drawNumericVlw(humidityValueBuf, 252, 235, MC_DATUM, TFT_CYAN, themeBg(), 0.546f, 3);
    vlwSet(lcd, UiFont24);
    lcd.setTextSize(1.04f, 1.04f);
    lcd.setTextDatum(MC_DATUM);
    setThemeTextColor(TFT_CYAN, themeBg());
    lcd.drawString("%", 301, 235);
    vlwRelease(lcd);
    lastPrintedMeteoHumidity = humidityValue;
  }
}

// AIR DATA: tre blocchi centrati (nome e sotto il valore), piu' grandi; tolte le righe CARB ICE / FOG CONDITIONS.
static const int AIR_LABEL_Y[3] = {104, 179, 254};   // Dew Point, Cloud Base AGL, Density Altitude
static const int AIR_VALUE_DY = 33;                   // valore sotto il nome
static void drawAirDataLabels() {
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1.4f, 1.4f);
  lcd.setTextDatum(MC_DATUM);
  setVividTextColor(TFT_CYAN);
  lcd.drawString("Dew Point", 180, AIR_LABEL_Y[0]);
  lcd.drawString("Cloud Base AGL", 180, AIR_LABEL_Y[1]);
  lcd.drawString("Density Altitude", 180, AIR_LABEL_Y[2]);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
}
static void drawAirDataValue(int row, const char *txt) {
  const int y = AIR_LABEL_Y[row] + AIR_VALUE_DY;
  lcd.fillRect(50, y - 19, 260, 38, themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize(1.45f, 1.45f);
  lcd.setTextDatum(MC_DATUM);
  setVividTextColor(TFT_WHITE);
  lcd.drawString(txt, 180, y);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
}

void drawAirDataScreen() {
  ForceColorThemeScope forceColor;   // AIR DATA sempre a colori
  lcd.fillScreen(themeBg());
  lastAirDataRedrawMillis = 0;
  lastPrintedAirDewPoint = -99999;
  lastPrintedAirCloudBase = -99999;
  lastPrintedAirDensityAlt = -99999;
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.5f) * THEME_TEXT_SCALE, (1.5f) * THEME_TEXT_SCALE);
  setVividTextColor(TFT_CYAN);
  lcd.setTextDatum(MC_DATUM);
  drawUiVlw("AIR DATA", 180, 45, MC_DATUM, themeVivid(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.04f) * THEME_TEXT_SCALE, (1.04f) * THEME_TEXT_SCALE);
  lcd.setTextDatum(MC_DATUM);
  setVividTextColor(TFT_YELLOW);
  lcd.drawString("ESTIMATE (indicative risk)", 180, 72);
  drawAirDataLabels();
  updateAirData();
  // Il meteo dal telefono non si scrive qui: a questa grandezza non si leggerebbe. Sta nella
  // pagina ONLINE WEATHER, primo passo del giro che parte con un tocco su questa pagina.
  if (metFresh()) drawUiVlw("tap: ONLINE WEATHER", 180, 322, MC_DATUM,
                            themeColor(TFT_DARKGREY), themeBg(), 0.40f);
}

// [FIX B] updateAirData: pulizia del messaggio d'errore quando il dato torna valido.
// Poiche' le etichette statiche vengono coperte dal messaggio di errore,
// sulla transizione invalid -> valid vanno ridisegnate esplicitamente.
void updateAirData() {
  if (currentScreen != AIR_DATA_SCREEN) return;
  ForceColorThemeScope forceColor;   // AIR DATA sempre a colori
  static bool lastAirDataValid = true;

  float temp = 0.0f;
  float humidity = 0.0f;
  float altitude = 0.0f;
  bool altitudeValid = false;
  bool meteoDataValid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    temp = meteo_temperature_c;
    humidity = meteo_humidity_percent;
    altitude = sharedFilteredAlt;
    altitudeValid = sharedAltitudeValid && isfinite(altitude);
    meteoDataValid = meteoValid && isfinite(temp) && isfinite(humidity);
    xSemaphoreGive(sensorDataMutex);
  }
  if (!altitudeValid || !meteoDataValid) {
    if (lastAirDataValid) lcd.fillRect(0, 85, 360, 245, themeBg());
    drawInvalidSensorMessage(!altitudeValid ? "ALT INVALID" : "METEO INVALID");
    lastAirDataValid = false;
    return;
  }
  if (!lastAirDataValid) {
    // Clear del messaggio d'errore + ridisegno delle etichette statiche.
    lcd.fillRect(0, 85, 360, 245, themeBg());
    drawAirDataLabels();
    lastPrintedAirDewPoint = -99999;
    lastPrintedAirCloudBase = -99999;
    lastPrintedAirDensityAlt = -99999;
    lastAirDataRedrawMillis = 0;
    lastAirDataValid = true;
  }
  float compTemp, compHumidity;
  compensateOutsideAirReading(temp, humidity, compTemp, compHumidity);
  float dewPoint = calculateDewPointSafe(compTemp, compHumidity);
  float cloudBase = calculateCloudBase(compTemp, dewPoint);
  float densityAlt = calculateDensityAltitude(compTemp, seaLevelPressure, altitude);
  int dpValue = isfinite(dewPoint) ? (int)round(dewPoint) : 0;
  int cbValue = isfinite(cloudBase) ? (int)round(cloudBase) : 0;
  int daValue = isfinite(densityAlt) ? (int)round(densityAlt) : 0;
  const bool dewPointChanged = dpValue != lastPrintedAirDewPoint;
  const bool cloudBaseChanged = cbValue != lastPrintedAirCloudBase;
  const bool densityAltChanged = daValue != lastPrintedAirDensityAlt;
  if (!dewPointChanged && !cloudBaseChanged && !densityAltChanged) return;
  unsigned long now = millis();
  if (lastAirDataRedrawMillis != 0 && now - lastAirDataRedrawMillis < AIR_DATA_REDRAW_INTERVAL_MS) return;
  lastAirDataRedrawMillis = now;
  char buf[32];
  if (dewPointChanged) { snprintf(buf, sizeof(buf), "%d C", dpValue); drawAirDataValue(0, buf); }
  if (cloudBaseChanged) { snprintf(buf, sizeof(buf), "%d m", cbValue); drawAirDataValue(1, buf); }
  if (densityAltChanged) { snprintf(buf, sizeof(buf), "%d ft", daValue); drawAirDataValue(2, buf); }
  lastPrintedAirDewPoint = dpValue;
  lastPrintedAirCloudBase = cbValue;
  lastPrintedAirDensityAlt = daValue;
}

// Come nello sketch di luglio, il valore visualizzato segue il passo della
// soglia audio selezionata: 0.2, 0.5, 0.8 oppure 1.0 m/s.
static float quantizeVarioForDisplay(float value) {
  int index = avThresholdIndex;
  if (index < 0 || index > 3) index = 0;
  const float step = avLiftThresholds[index];
  float quantized = roundf(value / step) * step;
  // Evita artefatti floating point e la visualizzazione di "-0.0".
  if (step >= 1.0f) quantized = roundf(quantized);
  else quantized = roundf(quantized * 10.0f) / 10.0f;
  if (quantized == 0.0f) quantized = 0.0f;
  return quantized;
}

static void formatVarioValue(float value, char *buffer, size_t bufferSize) {
  if (varioUpThreshold >= 1.0f) {
    snprintf(buffer, bufferSize, "%d", (int)roundf(value));
  } else {
    snprintf(buffer, bufferSize, "%.1f", value);
  }
}

// [FIX B] updateVarioData: sul ritorno a valido forza il redraw del blocco
// (che contiene gia' il fillRect dell'area) resettando i tracker.
// Quota sotto "m/s" (ciano come il titolo). Unita' come in SET QNH; "-- mt"/"-- ft" se non valida.
// Ridisegnata solo quando cambia valore o unita'. Area dedicata y 276..318: un tocco qui apre SET QNH.
static LGFX_Sprite varioAltSprite(&lcd);
static bool varioAltSpriteTried = false, varioAltSpriteOk = false;
const int VARIO_ALT_SPR_X = 40, VARIO_ALT_SPR_Y = 276, VARIO_ALT_SPR_W = 280, VARIO_ALT_SPR_H = 42;

static void updateVarioAltitude(bool valid, float altMeters) {
  const bool meters = altitudeUnitsIsMeters;
  const float shown = meters ? altMeters : altMeters * 3.28084f;
  int value = valid ? (int)lroundf(shown) : VARIO_ALT_INVALID;
  // Isteresi: il numero non salta avanti e indietro quando la quota oscilla attorno a x,5.
  if (valid && meters == lastPrintedVarioAltMeters && lastPrintedVarioAlt != VARIO_ALT_FORCE_REDRAW &&
      lastPrintedVarioAlt != VARIO_ALT_INVALID && fabsf(shown - (float)lastPrintedVarioAlt) < 0.65f) {
    value = lastPrintedVarioAlt;
  }
  if (value == lastPrintedVarioAlt && meters == lastPrintedVarioAltMeters) return;
  lastPrintedVarioAlt = value;
  lastPrintedVarioAltMeters = meters;

  if (!varioAltSpriteTried) {
    varioAltSpriteTried = true;
    varioAltSpriteOk = createSmallSprite(varioAltSprite, VARIO_ALT_SPR_W, VARIO_ALT_SPR_H);
  }
  // "ALT 1842 mt" centrata: ALT e unita' in ciano, solo il numero in giallo. Disegnata nello sprite
  // e spinta sul display in un colpo solo (niente sfarfallio); senza sprite si disegna direttamente.
  lgfx::LGFXBase &g = varioAltSpriteOk ? (lgfx::LGFXBase&)varioAltSprite : (lgfx::LGFXBase&)lcd;
  const int ox = varioAltSpriteOk ? 0 : VARIO_ALT_SPR_X;
  const int oy = varioAltSpriteOk ? 0 : VARIO_ALT_SPR_Y;
  const uint16_t bg = themeBg();
  if (varioAltSpriteOk) varioAltSprite.fillSprite(bg);
  else lcd.fillRect(VARIO_ALT_SPR_X, VARIO_ALT_SPR_Y, VARIO_ALT_SPR_W, VARIO_ALT_SPR_H, bg);

  char num[12];
  if (valid) snprintf(num, sizeof(num), "%d", value);
  else snprintf(num, sizeof(num), "--");
  const char *unit = meters ? " mt" : " ft";
  const float scale = 0.85f;
  const bool loaded = varioAltSpriteOk ? vlwSet(varioAltSprite, UiFont32) : vlwSet(lcd, UiFont32);
  if (loaded) g.setTextSize(scale, scale);
  else { g.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b); g.setTextSize(scale * 1.5f, scale * 1.5f); }
  const int wAlt = g.textWidth("ALT ");
  const int wNum = g.textWidth(num);
  const int wUnit = g.textWidth(unit);
  int x = ox + VARIO_ALT_SPR_W / 2 - (wAlt + wNum + wUnit) / 2;
  const int y = oy + VARIO_ALT_SPR_H / 2;
  g.setTextDatum(ML_DATUM);
  g.setTextColor(themeColor(TFT_CYAN), bg);
  g.drawString("ALT ", x, y);
  x += wAlt;
  g.setTextColor(themeVivid(TFT_YELLOW), bg);
  g.drawString(num, x, y);
  x += wNum;
  g.setTextColor(themeColor(TFT_CYAN), bg);
  g.drawString(unit, x, y);
  if (loaded) { if (varioAltSpriteOk) vlwRelease(varioAltSprite); else vlwRelease(lcd); }
  if (varioAltSpriteOk) { varioAltSprite.pushSprite(VARIO_ALT_SPR_X, VARIO_ALT_SPR_Y); drawPageLockOverlay(); }
}

void updateVarioData() {
  if (currentScreen != VARIOMETER_SCREEN) return;
  static bool lastVarioValid = true;

  float localVario = 0.0f;
  bool varioValid = false;
  float localAlt = 0.0f;
  bool altValid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    localVario = sharedVarioKalman;
    varioValid = sharedVarioValid;
    localAlt = sharedFilteredAlt;
    altValid = sharedAltitudeValid && isfinite(localAlt);
    xSemaphoreGive(sensorDataMutex);
  }
  updateVarioAltitude(altValid, localAlt);
  if (!varioValid) {
    // Alla prima entrata nello stato non valido toglie anche "m/s" (che sta sotto l'area del messaggio).
    if (lastVarioValid) lcd.fillRect(5, 90, 350, 182, themeBg());
    drawInvalidSensorMessage("VARIO INVALID");
    drawVarioSeparatorLine();
    lastVarioValid = false;
    return;
  }
  if (!lastVarioValid) {
    lastPrintedVario = -99999.0f;
    lastVarioDirection = 99;
    lastVarioValid = true;
  }
  float dispVario = quantizeVarioForDisplay(localVario);
  int direction = (localVario >= varioUpThreshold) ? 1 : ((localVario <= varioDownThreshold) ? -1 : 0);
  if (dispVario != lastPrintedVario || direction != lastVarioDirection) {
    lcd.fillRect(5, 90, 350, 182, themeBg());   // fino a y=272: la quota sotto non viene cancellata
    // Prima le frecce, poi i numeri: le cifre restano sempre in primo piano
    // e non vengono coperte dai triangoli.
    // Triangolo acceso a colori; quello spento resta come contorno da 1 px nello stesso grigio
    // delle barre spente del variometro nella schermata ALTIMETER
    const uint16_t outline = lcd.color565(42, 42, 42);
    if (direction > 0) lcd.fillTriangle(290, 105, 242, 205, 338, 205, themeVivid(TFT_GREEN));
    else lcd.drawTriangle(290, 105, 242, 205, 338, 205, outline);
    if (direction < 0) lcd.fillTriangle(70, 205, 22, 105, 118, 105, themeVivid(TFT_RED));
    else lcd.drawTriangle(70, 205, 22, 105, 118, 105, outline);
    char varioBuf[32];
    formatVarioValue(dispVario, varioBuf, sizeof(varioBuf));
    bool varioFontLoaded = vlwSet(lcd, VarioNumbers80);
    if (!varioFontLoaded) {
      lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
      lcd.setTextSize(1.8f, 1.8f);
    } else {
      lcd.setTextSize(1.2f, 1.2f);
    }
    setThemeTextColor(TFT_WHITE, TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString(varioBuf, 180, 180);   // centrato sul display
    if (varioFontLoaded) vlwRelease(lcd);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    setThemeTextColor(TFT_CYAN, TFT_BLACK);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString("m/s", 180, 238);   // subito sotto il valore del vario
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    drawVarioSeparatorLine();
    lastPrintedVario = dispVario;
    lastVarioDirection = direction;
  }
}

// ========== AVVISO RISCHIO GHIACCIO/NEBBIA (NON BLOCCANTE) ==========
// L'avviso e' una schermata normale (RISK_WARNING_SCREEN): mentre e' visibile il
// loop() continua a girare (lettura GPS, timer di volo, vario...). Si chiude con
// un tocco e si torna alla pagina che era visualizzata subito prima.

// Legge temperatura/umidita' dal BME280, le compensa e calcola i livelli di
// rischio. Ritorna false se non c'e' ancora una lettura valida.
static bool evaluateRiskLevels(int &carbRisk, int &fogRisk) {
  float rawTemp = 0.0f, rawHumidity = 0.0f;
  bool meteoOk = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    rawTemp = meteo_temperature_c;
    rawHumidity = meteo_humidity_percent;
    meteoOk = meteoValid && isfinite(rawTemp) && isfinite(rawHumidity) && rawHumidity > 0.0f;
    xSemaphoreGive(sensorDataMutex);
  }
  if (!meteoOk) return false;
  float compTemp, compHumidity;
  compensateOutsideAirReading(rawTemp, rawHumidity, compTemp, compHumidity);
  const float dewPoint = calculateDewPointSafe(compTemp, compHumidity);
  carbRisk = calculateCarbIceRisk(compTemp, compHumidity);
  fogRisk = calculateFogRisk(compTemp, dewPoint);
  riskWarningTempC = compTemp;
  riskWarningHumidity = compHumidity;
  return true;
}

// ---- Pagina WEATHER WARNING: una sola icona, scelta in base al rischio piu' alto ----
// Schermata dedicata sempre a colori (fondo bianco, come il quadrante di riferimento):
// ignora displayTheme e THEME_TEXT_SCALE. Livelli: 0 = basso, 1 = moderato, 2 = alto.

// ---- Scritta di stato lampeggiante (FOG, SERIOUS RISK, HAZE POSSIBLE, MUCH LONGER TAKE-OFF): ogni 0.5 s
// si cancella e si riscrive solo la riga dello stato, senza ridisegnare la pagina.
static bool stBlinkActive = false, stBlinkOn = true;
static char stBlinkTxt[28] = "";
static uint16_t stBlinkCol = TFT_WHITE;
static float stBlinkScale = 0.8f;
static int stBlinkY = 212;
static unsigned long stBlinkMs = 0;
static void statusBlinkSet(bool active, const char *txt, uint16_t col, float scale, int y) {
  stBlinkActive = active;
  if (!active) return;
  snprintf(stBlinkTxt, sizeof(stBlinkTxt), "%s", txt);
  stBlinkCol = col; stBlinkScale = scale; stBlinkY = y;
  stBlinkOn = true; stBlinkMs = millis();
}
void statusBlinkStep() {
  if (!stBlinkActive) return;
  if (currentScreen != RISK_WARNING_SCREEN && currentScreen != DENSITY_ALT_SCREEN) { stBlinkActive = false; return; }
  const unsigned long now = millis();
  if (now - stBlinkMs < 500UL) return;
  stBlinkMs = now;
  stBlinkOn = !stBlinkOn;
  ForceColorThemeScope forceColor;
  lcd.fillRect(42, stBlinkY - 15, 276, 30, TFT_BLACK);   // dentro l'arco
  if (stBlinkOn) drawUiVlw(stBlinkTxt, 180, stBlinkY, MC_DATUM, stBlinkCol, TFT_BLACK, stBlinkScale);
}

// ---- CARB ICE e VISIBILITY nello stile di DENSITY ALTITUDE: fondo nero, arco colorato con pallino bianco,
// icona e titolo in alto, valore grande al centro, stato, riquadro con consiglio e dati. Sempre a colori.
static void riskDaArcDot(float frac) {
  const float ang = (135.0f + 270.0f * constrain(frac, 0.0f, 1.0f)) * DEG_TO_RAD;
  const int dx = 180 + (int)lroundf(160.0f * cosf(ang)), dy = 180 + (int)lroundf(160.0f * sinf(ang));
  UT().fillCircle(dx, dy, 12, TFT_BLACK);
  UT().fillCircle(dx, dy, 9, TFT_WHITE);
}
// numero grande con "°C": numero a destra di x = 176, cerchietto e C a sinistra
static void riskDaBigDegrees(float v, int y) {
  char b[12];
  snprintf(b, sizeof(b), "%.0f", v);
  drawUiVlw(b, 172, y, MR_DATUM, TFT_WHITE, TFT_BLACK, 1.5f);
  UT().fillCircle(184, y - 17, 5, TFT_WHITE); UT().fillCircle(184, y - 17, 2, TFT_BLACK);
  drawUiVlw("C", 194, y, ML_DATUM, TFT_WHITE, TFT_BLACK, 1.5f);
}
static void riskDaBox(const char *l1, const char *l2) {
  UT().fillRoundRect(80, 240, 200, 62, 8, 0x0841);
  UT().drawRoundRect(80, 240, 200, 62, 8, TFT_WHITE);
  drawUiVlw(l1, 180, 259, MC_DATUM, TFT_WHITE, 0x0841, strlen(l1) > 19 ? 0.5f : 0.55f);
  drawUiVlw(l2, 180, 284, MC_DATUM, TFT_WHITE, 0x0841, 0.5f);
}
static void riskSnowflake(int x, int y, float r, uint16_t c) {
  for (int k = 0; k < 3; ++k) {
    const float a = k * 3.14159265f / 3.0f;
    const float ca = cosf(a), sa = sinf(a);
    UT().drawWideLine(x - r * ca, y - r * sa, x + r * ca, y + r * sa, 1.6f, c);
    for (int sd = -1; sd <= 1; sd += 2) {
      const float bx = x + sd * r * 0.55f * ca, by = y + sd * r * 0.55f * sa;
      for (int t = -1; t <= 1; t += 2) {
        const float b = a + t * 0.785f;
        UT().drawWideLine(bx, by, bx + sd * r * 0.35f * cosf(b), by + sd * r * 0.35f * sinf(b), 1.4f, c);
      }
    }
  }
}

static void drawCarbIceWarning(lgfx::LGFXBase &g) {
  (void)g;
  ForceColorThemeScope forceColor;
  const int level = riskWarningCarbLevel;
  const uint16_t cGreen = themeVivid(TFT_GREEN), cOrange = TFT_ORANGE, cRed = themeVivid(TFT_RED);
  const uint16_t cl = level >= 2 ? cRed : (level == 1 ? cOrange : cGreen);
  const float temp = riskWarningTempC, hum = riskWarningHumidity;
  const bool tempOk = isfinite(temp), humOk = isfinite(hum);
  UT().fillScreen(TFT_BLACK);
  fuelArc(UT(), 153, 167, 0.0f, 1.0f / 3.0f, cGreen);
  fuelArc(UT(), 153, 167, 1.0f / 3.0f, 2.0f / 3.0f, cOrange);
  fuelArc(UT(), 153, 167, 2.0f / 3.0f, 1.0f, cRed);
  fuelArc(UT(), 146, 147, 0.0f, 1.0f, lgfx::color565(154, 154, 154));   // arco fisso interno sottile
  UT().fillArc(180, 180, 172, 179, 0, 360, cl);   // bordo sempre, del colore del livello (verde = nessun rischio): la pagina copre tutto il display
  // pallino: dentro la fascia del livello, piu' avanti quanto piu' l'umidita' e' vicina alla soglia successiva
  float frac;
  const float h = humOk ? hum : 0.0f;
  if (level >= 2) frac = 2.0f / 3.0f + constrain((h - 80.0f) / 20.0f, 0.05f, 0.95f) / 3.0f;
  else if (level == 1) frac = 1.0f / 3.0f + constrain((h - 50.0f) / 30.0f, 0.05f, 0.95f) / 3.0f;
  else frac = constrain(h / 50.0f, 0.05f, 0.95f) / 3.0f;
  riskDaArcDot(frac);
  riskSnowflake(180, 46, 13.0f, cl);
  drawUiVlw("CARB ICE", 180, 76, MC_DATUM, TFT_CYAN, TFT_BLACK, 0.75f);
  drawUiVlw("RISK", 180, 100, MC_DATUM, TFT_CYAN, TFT_BLACK, 0.75f);
  if (tempOk) riskDaBigDegrees(temp, 142);
  else drawUiVlw("--", 180, 142, MC_DATUM, TFT_WHITE, TFT_BLACK, 1.5f);
  char buf[40];
  if (humOk) snprintf(buf, sizeof(buf), "Humidity %d %%", (int)lroundf(hum)); else snprintf(buf, sizeof(buf), "Humidity -- %%");
  drawUiVlw(buf, 180, 182, MC_DATUM, cl, TFT_BLACK, 0.65f);
  const char *stTxt = level >= 2 ? "SERIOUS RISK" : (level == 1 ? "MODERATE RISK" : "NO RISK");
  drawUiVlw(stTxt, 180, 212, MC_DATUM, cl, TFT_BLACK, 0.8f);
  statusBlinkSet(level >= 2, stTxt, cl, 0.8f, 212);   // rischio alto: SERIOUS RISK lampeggia
  const float dew = calculateDewPointSafe(temp, hum);
  char d2[32];
  if (isfinite(dew)) snprintf(d2, sizeof(d2), "DEW POINT %.0f C", dew); else snprintf(d2, sizeof(d2), "DEW POINT -- C");
  riskDaBox(level >= 2 ? "Apply carb heat" : (level == 1 ? "Check carb heat" : "No action needed"), d2);
  riskWarningDrawnTempTenth = tempOk ? (int)lroundf(temp) * 10 : -9999;
  riskWarningDrawnHumidity = humOk ? (int)lroundf(hum) : -9999;
}

// icone della visibilita' (disegnate): sole, sole velato, nuvola, righe di nebbia
static void riskVisIcon(int kind, int x, int y, uint16_t c) {
  if (kind == 0) {
    UT().fillCircle(x, y, 7, c);
    for (int k = 0; k < 8; ++k) {
      const float a = k * 0.785f;
      UT().drawWideLine(x + 11 * cosf(a), y + 11 * sinf(a), x + 15 * cosf(a), y + 15 * sinf(a), 1.5f, c);
    }
  } else if (kind == 1) {
    UT().fillArc(x, y - 3, 7, 10, 180, 360, c);
    UT().drawWideLine(x - 15, y + 4, x + 15, y + 4, 1.5f, c);
    UT().drawWideLine(x - 15, y + 10, x + 15, y + 10, 1.5f, c);
  } else if (kind == 2) {
    UT().fillArc(x - 6, y + 2, 5, 8, 90, 270, c);
    UT().fillArc(x + 1, y - 4, 7, 10, 180, 360, c);
    UT().fillArc(x + 9, y + 2, 5, 8, 270, 360, c);
    UT().fillArc(x + 9, y + 2, 5, 8, 0, 90, c);
    UT().drawWideLine(x - 12, y + 15, x + 12, y + 15, 1.5f, c);
  } else {
    UT().drawWideLine(x - 16, y - 8, x + 16, y - 8, 1.5f, c);
    UT().drawWideLine(x - 12, y, x + 12, y, 1.5f, c);
    UT().drawWideLine(x - 16, y + 8, x + 16, y + 8, 1.5f, c);
  }
}

static void drawFogWarning(lgfx::LGFXBase &g) {
  (void)g;
  ForceColorThemeScope forceColor;
  // Stati da calculateFogRisk(): 0 = GOOD VISIBILITY, 1 = LOW VISIBILITY, 2 = FOG;
  // con 0 e spread tra 4 e 8 C: HAZE POSSIBLE (solo informativa).
  const int visLevel = riskWarningFogLevel;
  const float temp = riskWarningTempC, hum = riskWarningHumidity;
  const bool tempOk = isfinite(temp), humOk = isfinite(hum);
  const float dew = calculateDewPointSafe(temp, hum);
  const bool haze = visLevel <= 0 && calculateHazePossible(temp, dew);
  const int state = visLevel >= 2 ? 3 : (visLevel == 1 ? 2 : (haze ? 1 : 0));   // 0 buona, 1 foschia, 2 ridotta, 3 nebbia
  const uint16_t cGreen = themeVivid(TFT_GREEN), cYel = themeVivid(TFT_YELLOW), cOrange = TFT_ORANGE, cRed = themeVivid(TFT_RED);
  const uint16_t cl = state == 3 ? cRed : (state == 2 ? cOrange : (state == 1 ? cYel : cGreen));
  UT().fillScreen(TFT_BLACK);
  fuelArc(UT(), 153, 167, 0.0f, 0.25f, cGreen);
  fuelArc(UT(), 153, 167, 0.25f, 0.5f, cYel);
  fuelArc(UT(), 153, 167, 0.5f, 0.75f, cOrange);
  fuelArc(UT(), 153, 167, 0.75f, 1.0f, cRed);
  fuelArc(UT(), 146, 147, 0.0f, 1.0f, lgfx::color565(154, 154, 154));   // arco fisso interno sottile
  UT().fillArc(180, 180, 172, 179, 0, 360, cl);   // bordo sempre, del colore dello stato: la pagina copre tutto il display
  // pallino dallo spread T - Td: oltre 14 C all'inizio, 8 / 4 / 2 C ai confini delle fasce, 0 C in fondo
  const float sp = (tempOk && isfinite(dew)) ? max(0.0f, temp - dew) : 20.0f;
  float frac;
  if (sp > 8.0f) frac = 0.25f * constrain((14.0f - sp) / 6.0f, 0.03f, 0.97f);
  else if (sp > 4.0f) frac = 0.25f + 0.25f * (8.0f - sp) / 4.0f;
  else if (sp > 2.0f) frac = 0.5f + 0.25f * (4.0f - sp) / 2.0f;
  else frac = 0.75f + 0.25f * constrain((2.0f - sp) / 2.0f, 0.03f, 0.97f);
  riskDaArcDot(frac);
  riskVisIcon(state, 180, 46, cl);
  drawUiVlw("VISIBILITY", 180, 82, MC_DATUM, TFT_CYAN, TFT_BLACK, 0.75f);
  drawUiVlw("Spread T - dew point", 180, 108, MC_DATUM, lgfx::color565(150, 170, 170), TFT_BLACK, 0.45f);
  if (tempOk && isfinite(dew)) riskDaBigDegrees(sp, 145);
  else drawUiVlw("--", 180, 145, MC_DATUM, TFT_WHITE, TFT_BLACK, 1.5f);
  char buf[40];
  if (humOk) snprintf(buf, sizeof(buf), "Humidity %d %%", (int)lroundf(hum)); else snprintf(buf, sizeof(buf), "Humidity -- %%");
  drawUiVlw(buf, 180, 182, MC_DATUM, cl, TFT_BLACK, 0.65f);
  static const char *const ST[4] = { "GOOD VISIBILITY", "HAZE POSSIBLE", "LOW VISIBILITY", "FOG" };
  drawUiVlw(ST[state], 180, 212, MC_DATUM, cl, TFT_BLACK, state == 3 ? 0.8f : 0.72f);
  statusBlinkSet(state == 3 || state == 1, ST[state], cl, state == 3 ? 0.8f : 0.72f, 212);   // FOG e HAZE POSSIBLE lampeggiano
  char l2[64];
  // Se il telefono ha mandato la visibilita', si aggiunge: e' un dato misurato sulla zona,
  // mentre temperatura e rugiada sono il sensore di bordo. Senza telefono resta come prima.
  const bool visTel = metVisValid && metFresh();
  if (tempOk && isfinite(dew) && visTel)
    snprintf(l2, sizeof(l2), "T %.0f - DEW %.0f - VIS %.0f km", temp, dew, metVisM / 1000.0f);
  else if (visTel)
    snprintf(l2, sizeof(l2), "VIS %.0f km", metVisM / 1000.0f);
  else if (tempOk && isfinite(dew)) snprintf(l2, sizeof(l2), "TEMP %.0f C - DEW %.0f C", temp, dew);
  else snprintf(l2, sizeof(l2), "TEMP -- C - DEW -- C");
  riskDaBox(state == 3 ? "Do not take off VFR" : (state >= 1 ? "Check METAR visibility" : "No action needed"), l2);
  riskWarningDrawnTempTenth = tempOk ? (int)lroundf(temp) * 10 : -9999;
  riskWarningDrawnHumidity  = humOk ? (int)lroundf(hum) : -9999;
}

static void drawRiskWarningScreenBody();
void drawRiskWarningScreen() {
  // disegno nello sprite e un solo invio al display: niente lampeggio quando cambia un valore
  const bool spr = ensureNavigationSprite();
  if (spr) uiTarget = &navigationSprite;
  drawRiskWarningScreenBody();
  uiTarget = nullptr;
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}
static void drawRiskWarningScreenBody() {
  // Un'icona sola: quella del rischio piu' alto (a parita' vince CARB ICE). Disegno diretto, come DENSITY ALTITUDE.
  const bool showCarb = riskWarningForcedPage == 1 ? true
                      : (riskWarningForcedPage == 2 ? false : riskWarningCarbLevel >= riskWarningFogLevel);
  if (showCarb) drawCarbIceWarning(UT());
  else drawFogWarning(UT());
}

// Avvio dell'avviso a schermo intero (accensione e ogni 10 minuti in volo): ghiaccio al carburatore,
// nebbia / visibilita' ridotta e anche HAZE POSSIBLE. Con piu' rischi le pagine si alternano ogni 3 s;
// un tocco chiude e torna alla pagina di prima (all'accensione: la pagina salvata).
// HAZE in volo solo se peggiora: dopo un avviso HAZE ne compare un altro solo se lo spread T - Td
// e' sceso di almeno HAZE_WORSEN_SPREAD_C rispetto all'ultimo avviso. Se la foschia sparisce si riparte da zero.
float hazeLastAlertSpread = NAN;
const float HAZE_WORSEN_SPREAD_C = 2.0f;
void startRiskWarning(int carbRisk, int fogRisk, bool autoClose, bool inFlight) {
  const float dew = calculateDewPointSafe(riskWarningTempC, riskWarningHumidity);
  const bool hazeNow = fogRisk <= 0 && calculateHazePossible(riskWarningTempC, dew);
  const float spread = riskWarningTempC - dew;
  if (!hazeNow) hazeLastAlertSpread = NAN;   // niente foschia (o gia' LOW VISIBILITY / FOG): si riparte
  bool haze = hazeNow;
  if (hazeNow && inFlight && isfinite(hazeLastAlertSpread) &&
      !(spread <= hazeLastAlertSpread - HAZE_WORSEN_SPREAD_C)) {
    haze = false;   // stessa foschia gia' segnalata e non peggiorata: in volo non la ripropone
  }
  if (haze && isfinite(spread)) hazeLastAlertSpread = spread;
  const bool visPage = fogRisk >= 1 || haze;
  if (carbRisk < 1 && !visPage) return;
  riskWarningCycle = carbRisk >= 1 && visPage;
  riskWarningForcedPage = carbRisk >= 1 ? 1 : 2;
  riskWarningStepNext = 0;
  if (!inFlight && !autoClose && riskWarningCycle) {
    // accensione: CARB ICE, poi con un tocco VISIBILITY, poi con un altro tocco la pagina salvata
    riskWarningCycle = false;
    riskWarningStepNext = 2;
  }
  riskWarningCycleSwitchMillis = millis();
  beginRiskWarningScreen(carbRisk, fogRisk, autoClose);
}

void beginRiskWarningScreen(int carbRisk, int fogRisk, bool autoClose) {
  if (currentScreen == RISK_WARNING_SCREEN) return;
  riskWarningAutoClose = autoClose;
  riskWarningShownMillis = millis();
  riskWarningPreviousScreen = currentScreen;
  riskWarningCarbLevel = carbRisk;
  riskWarningFogLevel = fogRisk;
  // Se il dito e' gia' appoggiato, quel tocco non deve chiudere subito l'avviso:
  // il loop lo ignora fino al rilascio.
  if (isTouching) {
    ignoreTouchUntilRelease = true;
    isTouching = false;
  }
  currentScreen = RISK_WARNING_SCREEN;
  drawRiskWarningScreen();
}

void dismissRiskWarningScreen() {
  riskWarningCycle = false;
  riskWarningStepNext = 0;
  riskPreviewActive = false;
  riskWarningForcedPage = 0;
  lastTouchMillis = millis();
  // Il prossimo controllo in volo riparte da qui (10 minuti dopo la chiusura).
  lastRuntimeRiskCheckMillis = millis();
  currentScreen = riskWarningPreviousScreen;
  // All'accensione subito dopo tocca a SET QNH: ridisegnare qui farebbe comparire
  // la pagina salvata per un istante prima della sequenza di avvio.
  if (!bootWizardWaiting()) redrawCurrentScreen();
}

// Tocco sulla parte bassa della pagina METEO: mostra a turno AIR DATA, CARB ICE e VISIBILITY
// (3 s ciascuna) con i valori reali del sensore. Non dipende dai livelli di rischio. Swipe = torna a METEO.
void startRiskPreview() {
  riskPreviewActive = true;
  riskWarningForcedPage = 0;              // 0 = AIR DATA, 1 = CARB ICE, 2 = VISIBILITY (FOG)
  riskPreviewLastSwitchMillis = millis();
  currentScreen = AIR_DATA_SCREEN;
  drawAirDataScreen();
}

// Swipe dall'anteprima: torna alla pagina METEO (sensor dashboard).
void endRiskPreview() {
  riskPreviewActive = false;
  riskWarningForcedPage = 0;
  lastTouchMillis = millis();
  currentScreen = METEO_SCREEN;
  for (int i = 0; i < numScreens; ++i) { if (screenOrder[i] == METEO_SCREEN) { currentScreenIndex = i; break; } }
  redrawCurrentScreen();
}

// Giro dalla METEO, a mano: ogni tocco o swipe passa alla pagina successiva
// AIR DATA -> DENSITY ALTITUDE -> CARB ICE RISK -> VISIBILITY -> SENSOR (METEO).
void riskPreviewNext() {
  riskPreviewActive = true;
  riskPreviewLastSwitchMillis = millis();
  if (currentScreen == AIR_DATA_SCREEN) {
    wxReturnScreen = -1;         // dal giro di AIR DATA si esce col giro, non col ritorno
    openOnlineWxPage();          // primo passo: il meteo dal telefono, a caratteri grandi
  } else if (currentScreen == ONLINE_WX_SCREEN) {
    openDensityAltPage();
  } else if (currentScreen == DENSITY_ALT_SCREEN) {
    int carbRisk = 0, fogRisk = 0;
    evaluateRiskLevels(carbRisk, fogRisk);
    riskWarningForcedPage = 1;
    beginRiskWarningScreen(carbRisk, fogRisk, false);
  } else if (currentScreen == RISK_WARNING_SCREEN && riskWarningForcedPage == 1) {
    riskWarningForcedPage = 2;
    drawRiskWarningScreen();
  } else {
    endRiskPreview();   // dopo VISIBILITY: torna a SENSOR
  }
  playTone(1200, 50);
}

// Demo: la sequenza parte appena avviata la demo e mostra gli 8 stadi (3 s ciascuno).
// Durante la sequenza la simulazione resta ferma all'istante 0 e la routine demo vera e
// propria parte alla fine. Un tocco o uno swipe salta il resto della sequenza.
static void demoRiskTourApplyStep() {
  const DemoRiskStage &st = DEMO_RISK_STAGES[demoRiskTourStep];
  riskWarningTempC = st.tempC;
  riskWarningHumidity = st.humidity;
  riskWarningCarbLevel = calculateCarbIceRisk(st.tempC, st.humidity);
  riskWarningFogLevel = calculateFogRisk(st.tempC, calculateDewPointSafe(st.tempC, st.humidity));
  riskWarningForcedPage = st.page;
}

// Fine della sequenza (completata, saltata o interrotta): la demo normale parte da qui.
void endDemoIntro() {
  demoRiskTourActive = false;
  demoRiskTourDone = true;
  demoIntroActive = false;
}

// Calcola alba/tramonto per la posizione demo: serve alla pagina SUNSET della sequenza.
static bool demoIntroPrepareSunset() {
  int h, m, sec, day, mon, yr;
  if (!getLocalClock(h, m, sec, day, mon, yr)) return false;
  updateSunTimes(DEMO_BASE_LAT, DEMO_BASE_LON, day, mon, yr, (int)(tzAutoOffsetSec / 60));
  if (!sunTimesValid || strlen(sunsetText) < 5) return false;
  return sunsetText[0] >= '0' && sunsetText[0] <= '9' && sunsetText[3] >= '0' && sunsetText[3] <= '9';
}

static void updateDemoRiskTour() {
  if (!demoMode) {
    if (demoRiskTourActive) {
      if (currentScreen == RISK_WARNING_SCREEN) dismissRiskWarningScreen();
      else if (currentScreen == SUNSET_SCREEN) { currentScreen = sunsetPreviousScreen; redrawCurrentScreen(); }
    }
    demoRiskTourActive = false;
    demoRiskTourDone = false;
    demoIntroActive = false;
    return;
  }
  if (!demoIntroActive) return;   // sequenza gia' conclusa: demo normale
  const unsigned long now = millis();
  if (demoRiskTourActive) {
    const bool onSunsetStep = demoRiskTourStep >= DEMO_RISK_STAGE_COUNT - 1;
    const ScreenState expected = onSunsetStep ? SUNSET_SCREEN : RISK_WARNING_SCREEN;
    if (currentScreen != expected) {   // interrotta da un altro avviso: si passa alla demo normale
      riskWarningForcedPage = 0;
      endDemoIntro();
      return;
    }
    if (now - demoRiskTourStepMillis < DEMO_RISK_STAGE_MS) return;
    demoRiskTourStep++;
    if (demoRiskTourStep >= DEMO_RISK_STAGE_COUNT) {   // fine: torna alla pagina di partenza
      endDemoIntro();
      lastTouchMillis = millis();
      currentScreen = sunsetPreviousScreen;
      if (!bootWizardWaiting()) redrawCurrentScreen();   // all'accensione tocca a SET QNH
      return;
    }
    demoRiskTourStepMillis = now;
    if (demoRiskTourStep == DEMO_RISK_STAGE_COUNT - 1) {   // ultimo stadio: SUNSET
      if (!demoIntroPrepareSunset()) {   // dati non disponibili: salta il sunset
        endDemoIntro();
        dismissRiskWarningScreen();
        return;
      }
      riskWarningForcedPage = 0;
      demoSunsetShown = true;            // niente secondo avviso sunset in demo
      sunsetPreviousScreen = riskWarningPreviousScreen;
      currentScreen = SUNSET_SCREEN;
      drawSunsetScreen(lcd, sunsetText);
      return;
    }
    demoRiskTourApplyStep();
    drawRiskWarningScreen();
    return;
  }
  if (demoRiskTourDone) return;
  if (currentScreen == RISK_WARNING_SCREEN || currentScreen == SUNSET_SCREEN ||
      currentScreen == ICT_ALERT_SCREEN || currentScreen == FLIGHT_END_SCREEN ||
      currentScreen == WIFI_LOGS_SCREEN || currentScreen == WIFI_LOGIN_SCREEN) {   // un altro avviso e' aperto: aspetta, poi rinuncia
    if (now - demoIntroRequestMillis >= DEMO_INTRO_GIVEUP_MS) endDemoIntro();
    return;
  }
  demoRiskTourStep = 0;
  demoRiskTourStepMillis = now;
  demoRiskTourApplyStep();
  demoRiskTourActive = true;
  beginRiskWarningScreen(riskWarningCarbLevel, riskWarningFogLevel, false);
}

// Da chiamare a ogni giro del loop principale. Fa due cose, entrambe senza
// bloccare nulla:
//  1) controllo iniziale: appena c'e' una lettura BME280 valida (max 3 s dopo
//     l'avvio) apre l'avviso se il rischio e' MEDIUM/HIGH;
//  2) in volo: ogni 10 minuti ricontrolla e ripropone l'avviso finche' le
//     condizioni persistono.
void checkRuntimeRiskWarning() {
  if (startupSequenceActive) return;
  updateDemoRiskTour();
  if (demoRiskTourActive) return;
  if (demoMode && !riskPreviewActive) return;
  // Anteprima dalla METEO, passo AIR DATA: dopo 3 s passa a CARB ICE
  if (riskPreviewActive && currentScreen == AIR_DATA_SCREEN) {
    return;   // giro a mano: si passa alla pagina successiva con un tocco o uno swipe
  }
  // Anteprima dalla METEO, passo DENSITY ALTITUDE: resta finche' non tocchi
  if (riskPreviewActive && currentScreen == DENSITY_ALT_SCREEN) return;
  if (riskPreviewActive && currentScreen != RISK_WARNING_SCREEN) {   // anteprima interrotta da altro (es. ICT)
    riskPreviewActive = false;
    riskWarningForcedPage = 0;
  }
  // Aggiorna la schermata CHECK CONDITIONS se uno dei livelli cambia mentre e' visibile.
  if (currentScreen == RISK_WARNING_SCREEN) {
    static unsigned long lastWarningPageSampleMillis = 0;
    const unsigned long now = millis();
    if (riskWarningCycle && now - riskWarningCycleSwitchMillis >= RISK_WARNING_CYCLE_MS) {
      riskWarningCycleSwitchMillis = now;   // alterna CARB ICE <-> VISIBILITY
      riskWarningForcedPage = (riskWarningForcedPage == 1) ? 2 : 1;
      drawRiskWarningScreen();
    }
    // in volo si chiude da sola: 5 s con una pagina, 6 s se due pagine si alternano (3 s ciascuna)
    if (riskWarningAutoClose &&
        now - riskWarningShownMillis >= (riskWarningCycle ? 2 * RISK_WARNING_CYCLE_MS : RISK_WARNING_AUTO_CLOSE_MS)) {
      dismissRiskWarningScreen();   // chiusura automatica dopo 5 s (il prossimo controllo tra 10 minuti)
      return;
    }
    if (riskPreviewActive) {
      const bool switchNow = false;   // giro a mano: le pagine cambiano solo con tocco o swipe
      if (switchNow) {
        riskPreviewLastSwitchMillis = now;
        if (riskWarningForcedPage == 2) {   // dopo VISIBILITY si torna ad AIR DATA e il giro ricomincia
          riskWarningForcedPage = 0;
          currentScreen = AIR_DATA_SCREEN;
          drawAirDataScreen();
          return;
        }
        riskWarningForcedPage = 2;
      }
      if (switchNow || now - lastWarningPageSampleMillis >= 1000UL) {
        lastWarningPageSampleMillis = now;
        int carbRisk = riskWarningCarbLevel, fogRisk = riskWarningFogLevel;
        const bool evalOk = evaluateRiskLevels(carbRisk, fogRisk);
        bool changed = false;
        if (evalOk) {
          const int tempTenth = isfinite(riskWarningTempC) ? (int)lroundf(riskWarningTempC) * 10 : -9999;   // gradi interi: come mostrati
          const int humInt = isfinite(riskWarningHumidity) ? (int)lroundf(riskWarningHumidity) : -9999;
          changed = carbRisk != riskWarningCarbLevel || fogRisk != riskWarningFogLevel ||
                    tempTenth != riskWarningDrawnTempTenth || humInt != riskWarningDrawnHumidity;
          riskWarningCarbLevel = carbRisk;
          riskWarningFogLevel = fogRisk;
        }
        if (switchNow || changed) drawRiskWarningScreen();   // anche la pagina FOG mostra temperatura e umidita'
      }
      return;
    }
    if (now - lastWarningPageSampleMillis >= 1000UL) {
      lastWarningPageSampleMillis = now;
      int carbRisk = 0, fogRisk = 0;
      if (evaluateRiskLevels(carbRisk, fogRisk)) {
        const int tempTenth = isfinite(riskWarningTempC) ? (int)lroundf(riskWarningTempC) * 10 : -9999;   // gradi interi: come mostrati
        const int humInt = isfinite(riskWarningHumidity) ? (int)lroundf(riskWarningHumidity) : -9999;
        const bool levelsChanged = carbRisk != riskWarningCarbLevel || fogRisk != riskWarningFogLevel;
        riskWarningCarbLevel = carbRisk;
        riskWarningFogLevel = fogRisk;
        // Entrambe le pagine si aggiornano quando cambiano temperatura o umidita' (lancetta/finestrella, righe FOG).
        if (levelsChanged || tempTenth != riskWarningDrawnTempTenth || humInt != riskWarningDrawnHumidity) {
          drawRiskWarningScreen();
        }
      }
    }
    return;
  }
  if (currentScreen == ICT_ALERT_SCREEN || currentScreen == AIRSPACE_ALERT_SCREEN ||
      currentScreen == DENSITY_ALT_SCREEN) return;

  if (startupRiskCheckPending) {
    int carbRisk = 0, fogRisk = 0;
    if (evaluateRiskLevels(carbRisk, fogRisk)) {
      startupRiskCheckPending = false;
      // Tutto verde (nessun rischio): all'accensione non si mostra niente e si va dritti alla
      // sequenza di avvio. L'avviso compare solo da MODERATE in su, su almeno uno dei due.
      if (carbRisk > 0 || fogRisk > 0)
        startRiskWarning(carbRisk, fogRisk, false, false);   // anche HAZE POSSIBLE; piu' rischi = pagine alternate
    } else if ((long)(millis() - startupRiskCheckDeadlineMillis) >= 0) {
      startupRiskCheckPending = false;  // nessuna lettura valida: nessun avviso
    }
    return;
  }

  const bool flightActive = flightTimerRunning || automaticFlightRunning;
  if (!flightActive || currentScreen == FLIGHT_END_SCREEN) {
    runtimeRiskFlightActive = false;
    return;
  }
  unsigned long now = millis();
  if (!runtimeRiskFlightActive) {
    runtimeRiskFlightActive = true;
    lastRuntimeRiskCheckMillis = 0; // primo controllo immediato all'inizio del volo
  }
  if (alertsDeferred()) return;   // vicino alla meta: il controllo resta in attesa fino a 5 km
  if (lastRuntimeRiskCheckMillis != 0 &&
      now - lastRuntimeRiskCheckMillis < RUNTIME_RISK_CHECK_INTERVAL_MS) return;
  lastRuntimeRiskCheckMillis = now;

  int carbRisk = 0, fogRisk = 0;
  if (!evaluateRiskLevels(carbRisk, fogRisk)) return;
  startRiskWarning(carbRisk, fogRisk, true, true);   // in volo: HAZE solo se peggiora
}

// Avviso tramonto: 30 minuti prima del tramonto mostra la pagina SUNSET.
// Si chiude con un tocco e si torna alla pagina precedente.
// Una volta al giorno (per data locale) non si ripete.
void checkSunsetAlert() {
  if (startupSequenceActive) return;
  if (demoMode && demoIntroActive) return;   // in demo il sunset fa parte della sequenza iniziale
  if (currentScreen == SUNSET_SCREEN) return;
  if (currentScreen == RISK_WARNING_SCREEN || currentScreen == ICT_ALERT_SCREEN ||
      currentScreen == FLIGHT_END_SCREEN || currentScreen == WIFI_LOGS_SCREEN ||
      currentScreen == WIFI_LOGIN_SCREEN || currentScreen == AIRSPACE_ALERT_SCREEN ||
      currentScreen == DENSITY_ALT_SCREEN) return;
  if (alertsDeferred()) return;   // vicino alla meta: rimandato

  const unsigned long now = millis();
  const unsigned long checkInterval = demoMode ? 250UL : SUNSET_CHECK_INTERVAL_MS;
  if (lastSunsetCheckMillis != 0 &&
      now - lastSunsetCheckMillis < checkInterval) return;
  lastSunsetCheckMillis = now;

  int localHour, localMinute, localSecond, localDay, localMonth, localYear;
  if (!getLocalClock(localHour, localMinute, localSecond, localDay, localMonth, localYear)) return;

  const int dayKey = localYear * 10000 + localMonth * 100 + localDay;
  if ((!demoMode && sunsetShownDayKey == dayKey) ||
      (demoMode && demoSunsetShown)) return;

  // Posizione: prima il GPS, altrimenti la home salvata.
  double lat = 0.0, lon = 0.0;
  bool positionValid = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    if (gpsDataValid) {
      lat = gps_latitude;
      lon = gps_longitude;
      positionValid = true;
    }
    xSemaphoreGive(sensorDataMutex);
  }
  if (!positionValid && homePositionValid) {
    lat = homeLatitude;
    lon = homeLongitude;
    positionValid = true;
  }
  if (!positionValid) return;

  updateSunTimes(lat, lon, localDay, localMonth, localYear, (int)(tzAutoOffsetSec / 60));   // minuti
  if (!sunTimesValid) return;

  const int len = (int)strlen(sunsetText);
  if (len < 5) return;
  if (sunsetText[0] < '0' || sunsetText[0] > '9') return;
  if (sunsetText[3] < '0' || sunsetText[3] > '9') return;
  const int sunsetH = (sunsetText[0] - '0') * 10 + (sunsetText[1] - '0');
  const int sunsetM = (sunsetText[3] - '0') * 10 + (sunsetText[4] - '0');
  if (sunsetH < 0 || sunsetH > 23 || sunsetM < 0 || sunsetM > 59) return;

  const int sunsetMinutes = sunsetH * 60 + sunsetM;
  if (demoMode && !demoSunsetClockInitialized) {
    demoSunsetBaseMinute = sunsetMinutes - 31;
    demoSunsetClockStartMillis = millis();
    demoSunsetClockInitialized = true;
  }
  int nowMinutes = localHour * 60 + localMinute;
  if (demoMode && demoSunsetClockInitialized) {
    nowMinutes = demoSunsetBaseMinute +
                 (int)((millis() - demoSunsetClockStartMillis) / 1000UL);
  }
  const int diff = sunsetMinutes - nowMinutes;
  if (diff < 0 || diff > SUNSET_ALERT_MINUTES_BEFORE) return;

  // Mostra la pagina SUNSET.
  if (demoMode) demoSunsetShown = true;
  else sunsetShownDayKey = dayKey;
  sunsetPreviousScreen = currentScreen;
  // Se il dito è già appoggiato, quel tocco non deve chiuderla subito.
  if (isTouching) { ignoreTouchUntilRelease = true; isTouching = false; }
  currentScreen = SUNSET_SCREEN;
  drawSunsetScreen(lcd, sunsetText);
  playTone(1500, 80);
}

void drawIctScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.2f) * THEME_TEXT_SCALE, (1.2f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  drawUiVlw("Instrument", 180, 42, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  drawUiVlw("Check Timer", 180, 70, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  char b[16];
  snprintf(b, sizeof(b), "%d", demoMode ? DEMO_ICT_DURATION_SECONDS : ictMinutes);
  drawNumericVlw(b, 180, 145, MC_DATUM, TFT_WHITE, themeBg(), 0.9f);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.drawString(demoMode ? "sec" : "min", 180, 205);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.drawRect(25, 220, 90, 55, themeColor(TFT_GREEN));
  lcd.drawRect(245, 220, 90, 55, themeColor(TFT_GREEN));
  setThemeTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  lcd.drawString("-", 70, 247);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString("+", 290, 247);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  const uint16_t stateColor = TFT_GREEN;
  lcd.drawRect(85, 290, 190, 45, themeColor(stateColor));
  setThemeTextColor(stateColor, TFT_BLACK);
  lcd.setTextSize(0.9f * THEME_TEXT_SCALE, 0.9f * THEME_TEXT_SCALE);
  lcd.drawString(ictEnabled ? "TIMER: ON" : "TIMER: OFF", 180, 312);
}

void drawIctAlertScreen() {
  // Avviso ICT fisso (niente lampeggio): "Check" / "Flight" / "Instruments" in nero su 3 righe,
  // stesso carattere della scritta GPS TIME / UNAVAILABLE. Dura ICT_ALERT_DURATION_MS (8 s).
  if (ictAlertStaticDrawn) return;
  lcd.fillScreen(TFT_WHITE);
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  lcd.setTextSize(1.0f, 1.0f);
  lcd.setTextColor(TFT_BLACK, TFT_WHITE);
  lcd.drawString("Check", 180, 125);        // tre righe centrate come prima: Check / Flight / Instruments
  lcd.drawString("Flight", 180, 165);
  lcd.drawString("Instruments", 180, 205);
  ictAlertStaticDrawn = true;
}

void beginIctAlertScreen() {
  ictAlertPreviousScreen = currentScreen;
  ictAlertStaticDrawn = false;
  ictAlertTextVisible = true;
  ictAlertUseItalian = true;
  ictAlertLastBlinkMillis = millis() - 400UL;
  currentScreen = ICT_ALERT_SCREEN;
  drawIctAlertScreen();
}

void dismissIctAlertScreen() {
  ictAlarmActive = false;  // il tocco ferma subito anche il suono
  ictStartMillis = millis();
  currentScreen = ictAlertPreviousScreen;
  ictAlertStaticDrawn = false;
  redrawCurrentScreen();
}

// Allarme sonoro ICT NON bloccante: la sequenza (10 ripetizioni da 3 suoni, come
// prima) viene suonata un suono per volta dal loop(), cosi' il touch resta attivo
// e l'avviso si chiude subito al tocco, anche durante il suono.
void startIctCheckAlarm() {
  ictAlarmActive = true;
  ictAlarmStep = 0;
  ictAlarmNextMillis = 0;
}

void handleIctAlarm() {
  if (!ictAlarmActive) return;
  if (currentScreen != ICT_ALERT_SCREEN || ictAlarmStep >= 30) {
    ictAlarmActive = false;
    return;
  }
  if ((long)(millis() - ictAlarmNextMillis) < 0) return;
  switch (ictAlarmStep % 3) {
    case 0: playTone(2200, 80);  silenceAudio(); ictAlarmNextMillis = millis() + 10;  break;
    case 1: playTone(1200, 80);  silenceAudio(); ictAlarmNextMillis = millis() + 10;  break;
    default: playTone(2200, 120); silenceAudio(); ictAlarmNextMillis = millis() + 120; break;
  }
  ++ictAlarmStep;
}

void updateIctTimer() {
  if (bleRole == BLE_ROLE_SLAVE) return; // avviso ICT comandato dal master
  unsigned long now = millis();
  if (currentScreen == ICT_ALERT_SCREEN) {
    if (now - ictAlertStartMillis >= ICT_ALERT_DURATION_MS) dismissIctAlertScreen();
    return;
  }
  if (!ictEnabled || !automaticFlightRunning) return;
  const unsigned long duration = demoMode ? DEMO_ICT_DURATION_MS : (unsigned long)ictMinutes * 60000UL;
  if (now - ictStartMillis >= duration) {
    if (alertsDeferred()) return;   // vicino alla meta: l'avviso resta in attesa fino a 5 km
    ictAlertStartMillis = millis();
    beginIctAlertScreen();   // prima l'avviso a schermo, poi il suono (non bloccante)
    startIctCheckAlarm();
  }
}

void clearFlightLogMetrics() {
  flightMaxAltitudeMeters = 0.0f;
  flightMaxClimbRate = 0.0f;
  flightMaxSinkRate = 0.0f;
  flightMaxGpsSpeedKmh = 0.0f;
  flightMinTempC = 0.0f;
  flightMaxTempC = 0.0f;
  flightMinHumidity = 0.0f;
  flightMaxHumidity = 0.0f;
  flightEndTempC = 0.0f;
  flightEndHumidity = 0.0f;
  flightEndEnvironmentValid = false;
  flightTotalDistanceKm = 0.0f;
  flightMinG = 1.0f;
  flightMaxG = 1.0f;
  flightEnvironmentValid = false;
  flightAltitudeMetricValid = false;
  flightLogMetricsStarted = false;
  flightStatsLastUpdateMillis = 0;
  strcpy(flightDate, "--/--/----");
}

void startFlightLogMetrics(unsigned long now) {
  clearFlightLogMetrics();
  flightMinG = 1.0f;   // G-meter rimosso: i campi del log restano a 1.0 g
  flightMaxG = 1.0f;
  float alt = 0.0f, temp = NAN, humidity = NAN;
  bool altitudeValid = false;
  bool environmentValid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    alt = sharedFilteredAlt;
    altitudeValid = sharedAltitudeValid && isfinite(alt);
    temp = meteo_temperature_c;
    humidity = meteo_humidity_percent;
    environmentValid = meteoValid && isfinite(temp) && isfinite(humidity) &&
                        humidity >= 0.0f && humidity <= 100.0f;
    xSemaphoreGive(sensorDataMutex);
  }
  if (altitudeValid) {
    flightMaxAltitudeMeters = alt;
    flightAltitudeMetricValid = true;
  }
  if (environmentValid && isfinite(temp) && isfinite(humidity)) {
    flightMinTempC = flightMaxTempC = temp;
    flightMinHumidity = flightMaxHumidity = humidity;
    flightEnvironmentValid = true;
  }
  flightLogMetricsStarted = true;
  flightStatsLastUpdateMillis = now;
  int localHour, localMinute, localSecond, localDay, localMonth, localYear;
  if (getLocalClock(localHour, localMinute, localSecond, localDay, localMonth, localYear)) {
    snprintf(flightDate, sizeof(flightDate), "%02d/%02d/%04d", localDay, localMonth, localYear);
  } else {
    strcpy(flightDate, "--/--/----");
  }
}

// [FIX C] L'inizializzazione (startFlightLogMetrics) avviene ora PRIMA di
// leggere/confrontare l'accelerometro: in precedenza la prima lettura G di
// un volo veniva scartata perche' clearFlightLogMetrics() (chiamata da
// startFlightLogMetrics) la sovrascriveva subito dopo. Con questo ordine la
// funzione e' corretta anche se in futuro automaticFlightRunning venisse
// impostato a true senza chiamare esplicitamente startFlightLogMetrics().
void updateFlightLogMetrics(unsigned long now, float speedKmh, bool gpsValid) {
  if (!automaticFlightRunning) return;
  if (!flightLogMetricsStarted) startFlightLogMetrics(now);
  if (flightStatsLastUpdateMillis != 0) {
    unsigned long dtMs = now - flightStatsLastUpdateMillis;
    if (gpsValid && speedKmh >= 0.0f && dtMs <= 2000UL) {
      flightTotalDistanceKm += speedKmh * ((float)dtMs / 3600000.0f);
    }
  }
  flightStatsLastUpdateMillis = now;
  if (gpsValid && speedKmh > flightMaxGpsSpeedKmh) flightMaxGpsSpeedKmh = speedKmh;
  float alt = NAN, vario = NAN, temp = NAN, humidity = NAN;
  bool altitudeValid = false;
  bool environmentValid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    alt = sharedFilteredAlt;
    altitudeValid = sharedAltitudeValid && isfinite(alt);
    vario = sharedVarioKalman;
    temp = meteo_temperature_c;
    humidity = meteo_humidity_percent;
    environmentValid = meteoValid && isfinite(temp) && isfinite(humidity) &&
                        humidity >= 0.0f && humidity <= 100.0f;
    xSemaphoreGive(sensorDataMutex);
  }
  if (altitudeValid && (!flightAltitudeMetricValid || alt > flightMaxAltitudeMeters)) {
    flightMaxAltitudeMeters = alt;
    flightAltitudeMetricValid = true;
  }
  if (isfinite(vario) && vario > flightMaxClimbRate) flightMaxClimbRate = vario;
  if (isfinite(vario) && vario < flightMaxSinkRate) flightMaxSinkRate = vario;
  if (environmentValid && isfinite(temp) && isfinite(humidity)) {
    if (!flightEnvironmentValid) {
      flightMinTempC = flightMaxTempC = temp;
      flightMinHumidity = flightMaxHumidity = humidity;
      flightEnvironmentValid = true;
    } else {
      if (temp < flightMinTempC) flightMinTempC = temp;
      if (temp > flightMaxTempC) flightMaxTempC = temp;
      if (humidity < flightMinHumidity) flightMinHumidity = humidity;
      if (humidity > flightMaxHumidity) flightMaxHumidity = humidity;
    }
  }
}

void onFlightEnd() {
  float temp = NAN;
  float humidity = NAN;
  bool valid = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    temp = meteo_temperature_c;
    humidity = meteo_humidity_percent;
    valid = meteoValid && isfinite(temp) && isfinite(humidity) &&
            humidity >= 0.0f && humidity <= 100.0f;
    xSemaphoreGive(sensorDataMutex);
  }
  flightEndEnvironmentValid = valid;
  if (valid) {
    compensateOutsideAirReading(temp, humidity, flightEndTempC, flightEndHumidity);
  }
  if (automaticFlightAccumMillis > 0 && bleRole != BLE_ROLE_SLAVE) {
    saveCurrentFlightLog();
  }
  trackFinish();   // chiude la traccia GPS e tiene solo quelle dei voli presenti nei log
}

void dismissFlightEndScreen() {
  lastTouchMillis = millis();
  currentScreen = (flightEndPreviousScreen == FLIGHT_END_SCREEN)
                  ? AUTOMATIC_TIMER_SCREEN : flightEndPreviousScreen;
  for (int i = 0; i < numScreens; ++i) {
    if (screenOrder[i] == currentScreen) {
      currentScreenIndex = i;
      break;
    }
  }
  saveScreenPreference();
  redrawCurrentScreen();
  playTone(1200, 50);
}

// Riga START/STOP con font VLW antialiased (stessa grandezza del vecchio FreeSansBold9pt x1.25).
void drawStartStopLine(const char *text, int centerY) {
  const bool loaded = vlwSet(lcd, ClockFont22);
  if (loaded) {
    lcd.setTextSize(1.0f, 1.0f);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString(text, 180, centerY - 3 - (int)lroundf(ClockFont22_DIGIT_CENTER_DY));
    vlwRelease(lcd);
  } else {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    lcd.setTextSize((1.25f) * THEME_TEXT_SCALE, (1.25f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(MC_DATUM);
    lcd.drawString(text, 180, centerY);
  }
}

void drawTimerGroup(const char *timer, int centerY, uint16_t fg, float scale, int bg) {   // bg -1 = sfondo del tema
  // Font VLW antialiased (stessa forma e grandezza del vecchio FreeSansBold24pt x1.25).
  // Il centro delle cifre resta dove era con il vecchio font (circa 11 px sopra centerY).
  const bool loaded = vlwSet(lcd, ClockFont60);
  lcd.setTextDatum(MC_DATUM);
  lcd.setTextColor(displayTheme == THEME_COLOR ? fg : TFT_BLACK, bg < 0 ? themeBg() : (uint16_t)bg);
  if (loaded) {
    lcd.setTextSize(1.0f, 1.0f);
    lcd.drawString(timer, 180, centerY - 11 - (int)lroundf(ClockFont60_DIGIT_CENTER_DY));
    vlwRelease(lcd);
  } else {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
    lcd.setTextSize(scale, scale);
    lcd.drawString(timer, 180, centerY);
  }
}

void drawFlightTimerScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.15f) * THEME_TEXT_SCALE, (1.15f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  drawUiVlw("MANUAL", 180, 45, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  drawUiVlw("FLIGHT TIMER", 180, 75, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  unsigned long elapsed = flightTimerAccumMillis;
  if (flightTimerRunning) elapsed += millis() - flightTimerStartMillis;
  unsigned long sec = elapsed / 1000UL;
  char timer[16];
  snprintf(timer, sizeof(timer), "%02lu:%02lu:%02lu", sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
  drawTimerGroup(timer, 140, TFT_WHITE, 1.25f * THEME_TEXT_SCALE);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.25f) * THEME_TEXT_SCALE, (1.25f) * THEME_TEXT_SCALE);
  char line[32];
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  snprintf(line, sizeof(line), "START: %.5s", flightStartUtc);
  drawStartStopLine(line, 190);
  snprintf(line, sizeof(line), "STOP:  %.5s", flightStopUtc);
  drawStartStopLine(line, 215);
  uint16_t c = flightTimerRunning ? TFT_GREEN : TFT_RED;
  lcd.drawRect(30, 235, 95, 45, themeColor(TFT_GREEN));
  lcd.drawRect(132, 235, 95, 45, themeColor(TFT_RED));
  lcd.drawRect(235, 235, 95, 45, themeColor(TFT_ORANGE));
  // Scritte in font VLW antialiased (lisce, niente pixel) e piu' grandi, ma
  // dentro i riquadri 95x45.
  drawUiVlw("Start", 77, 257, MC_DATUM, themeColor(TFT_GREEN), themeBg(), 0.92f);
  drawUiVlw("Stop", 179, 257, MC_DATUM, themeColor(TFT_RED), themeBg(), 0.92f);
  drawUiVlw("Reset", 282, 257, MC_DATUM, themeColor(TFT_ORANGE), themeBg(), 0.92f);
  // Stato in VLW; leggermente alzato e limitato in larghezza dal bordo tondo del display.
  drawUiVlw(flightTimerRunning ? "MANUAL: RUNNING" : "MANUAL: STOPPED", 180, 298,
            MC_DATUM, themeColor(c), themeBg(), 0.8f);
}

void updateManualFlightTimerScreen() {
  if (currentScreen != FLIGHT_TIMER_SCREEN) return;
  unsigned long elapsed = flightTimerAccumMillis;
  if (flightTimerRunning) elapsed += millis() - flightTimerStartMillis;
  unsigned long sec = elapsed / 1000UL;
  char timer[16];
  snprintf(timer, sizeof(timer), "%02lu:%02lu:%02lu", sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
  static unsigned long lastSecond = 0xFFFFFFFFUL;
  static bool lastRunning = false;
  if (sec == lastSecond && flightTimerRunning == lastRunning) return;
  lastSecond = sec;
  lastRunning = flightTimerRunning;
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold24pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  drawTimerGroup(timer, 140, TFT_WHITE, 1.25f * THEME_TEXT_SCALE);
}

void updateFlightTimer() {
  // Lo slave non calcola autonomamente il timer, ma aggiorna la visualizzazione
  // con lo stato RUNNING/STOPPED ricevuto dal master.
  const bool isBleSlave = (bleRole == BLE_ROLE_SLAVE);
  if (!isBleSlave) {
    float speed = 0.0f;
    bool valid = false;
    double curLat = 0.0, curLon = 0.0;
    if (xSemaphoreTake(sensorDataMutex, (TickType_t)2) == pdTRUE) {
      speed = gps_ground_speed_kmh;
      valid = gpsDataValid;
      curLat = gps_latitude;
      curLon = gps_longitude;
      xSemaphoreGive(sensorDataMutex);
    }
    if (valid && automaticFlightRunning) lubberReanchorIfNeeded(curLat, curLon);
    unsigned long now = millis();
    // Anche la telemetria demo alimenta i log di volo simulati.
    if (automaticFlightRunning) updateFlightLogMetrics(now, speed, valid);
    if (automaticFlightRunning) trackSample(now, valid, curLat, curLon, speed);   // traccia GPS (KML)

    if (!valid) {
      // GPS perso: il timer automatico NON deve chiudere il volo.
      // Il volo continua a contare; la chiusura (atterraggio) viene valutata
      // solo con GPS valido e velocita' sotto soglia per SPEED_STOP_DEBOUNCE_MS.
      speedAboveStartSince = 0;
      speedBelowStopSince = 0;  // evita che un conteggio atterraggio parziale prosegua al rientro del GPS
      gpsInvalidFlightSince = 0;
    } else if (!automaticFlightRunning) {
      gpsInvalidFlightSince = 0;
      speedBelowStopSince = 0;
      if (speed >= TAKEOFF_SPEED_START_KMH) {
        if (speedAboveStartSince == 0) speedAboveStartSince = now;
        if (now - speedAboveStartSince >= SPEED_START_DEBOUNCE_MS) {
          automaticFlightRunning = true;
          automaticFlightStartMillis = now;
          automaticFlightAccumMillis = 0;
          if (ictEnabled) ictStartMillis = now;
          startFlightLogMetrics(now);
          lubberLegLat = curLat; lubberLegLon = curLon; lubberLegValid = true;  // tratta decollo -> casa
          lubberActive = true;
          getLocalTimeText(automaticStartUtc, sizeof(automaticStartUtc));
          snprintf(automaticStopUtc, sizeof(automaticStopUtc), "--:--:--");  // nuovo volo: azzera lo STOP vecchio
          trackStart();   // registrazione della traccia GPS
          speedAboveStartSince = 0;
          playTone(1500, 80);
        }
      } else speedAboveStartSince = 0;
    } else {
      gpsInvalidFlightSince = 0;
      speedAboveStartSince = 0;
      const float stopSpeedThreshold = demoMode
                                        ? DEMO_FLIGHT_STOP_SPEED_KMH
                                        : LANDING_SPEED_STOP_KMH;
      if (speed <= stopSpeedThreshold) {
        if (speedBelowStopSince == 0) speedBelowStopSince = now;
        if (now - speedBelowStopSince >= SPEED_STOP_DEBOUNCE_MS) {
          automaticFlightAccumMillis += now - automaticFlightStartMillis;
          automaticFlightRunning = false;
          flightStatsLastUpdateMillis = 0;
          getLocalTimeText(automaticStopUtc, sizeof(automaticStopUtc));
          speedBelowStopSince = 0;
          onFlightEnd();
          if (!demoTourActive) {   // rotazione demo: la pagina FLIGHT END arriva al suo turno
            if (currentScreen != FLIGHT_END_SCREEN) {
              flightEndPreviousScreen = (currentScreen == ICT_ALERT_SCREEN)
                                          ? ictAlertPreviousScreen : currentScreen;
            }
            currentScreen = FLIGHT_END_SCREEN;
            playTone(900, 80);
            drawFlightEndScreen();
          }
        }
      } else speedBelowStopSince = 0;
    }
  } // fine logica locale GPS/atterraggio; il rendering vale anche per lo slave

  static unsigned long lastTimerDrawSecond = 0xFFFFFFFFUL;
  unsigned long elapsed = automaticFlightAccumMillis;
  if (automaticFlightRunning) elapsed += millis() - automaticFlightStartMillis;
  unsigned long shownSecond = elapsed / 1000UL;
  static bool lastTimerDrawRunning = false;
  // Se START/STOP cambiano mentre la pagina e' visibile (nuovo volo, o valori
  // ricevuti dal master sullo slave) ridisegna tutta la pagina, non solo il numerico.
  static char lastDrawnStartUtc[16] = "";
  static char lastDrawnStopUtc[16] = "";
  if (currentScreen == AUTOMATIC_TIMER_SCREEN &&
      (strncmp(lastDrawnStartUtc, automaticStartUtc, sizeof(lastDrawnStartUtc)) != 0 ||
       strncmp(lastDrawnStopUtc, automaticStopUtc, sizeof(lastDrawnStopUtc)) != 0)) {
    strncpy(lastDrawnStartUtc, automaticStartUtc, sizeof(lastDrawnStartUtc) - 1);
    lastDrawnStartUtc[sizeof(lastDrawnStartUtc) - 1] = '\0';
    strncpy(lastDrawnStopUtc, automaticStopUtc, sizeof(lastDrawnStopUtc) - 1);
    lastDrawnStopUtc[sizeof(lastDrawnStopUtc) - 1] = '\0';
    lastTimerDrawSecond = shownSecond;
    lastTimerDrawRunning = automaticFlightRunning;
    drawAutomaticFlightTimerScreen();
  } else if (currentScreen == AUTOMATIC_TIMER_SCREEN &&
      (shownSecond != lastTimerDrawSecond || automaticFlightRunning != lastTimerDrawRunning)) {
    lastTimerDrawSecond = shownSecond;
    lastTimerDrawRunning = automaticFlightRunning;
    updateAutomaticFlightTimerScreen();
  }
  if (currentScreen == FLIGHT_TIMER_SCREEN) updateManualFlightTimerScreen();
}

void drawAutomaticFlightTimerScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.25f) * THEME_TEXT_SCALE, (1.25f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  drawUiVlw("AUTOMATIC", 180, 48, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  drawUiVlw("FLIGHT TIMER", 180, 78, MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  unsigned long elapsed = automaticFlightAccumMillis;
  if (automaticFlightRunning) elapsed += millis() - automaticFlightStartMillis;
  unsigned long sec = elapsed / 1000UL;
  char timer[16];
  snprintf(timer, sizeof(timer), "%02lu:%02lu:%02lu", sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
  drawTimerGroup(timer, 150, automaticFlightRunning ? TFT_GREEN : TFT_RED,
                 1.25f * THEME_TEXT_SCALE);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.25f) * THEME_TEXT_SCALE, (1.25f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  char line[40];
  snprintf(line, sizeof(line), "START: %.5s", automaticStartUtc);
  drawStartStopLine(line, 215);
  snprintf(line, sizeof(line), "STOP:  %.5s", automaticStopUtc);
  drawStartStopLine(line, 240);
  lcd.drawRect(85, 275, 190, 45, themeColor(TFT_YELLOW));
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  drawUiVlw("FLIGHT LOG", 180, 298, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.90f);
}

// ---------- Icone del FLIGHT LOG (14x14 px, centrate in x, y) ----------
// 0 orologio, 1 percorso, 2 carburante, 3 aereo, 4 montagne, 5 salita, 6 termometro, 7 goccia, 8 satellite (ora)
static void logRotRect(float cx, float cy, float l, float w, uint16_t c) {   // rettangolo ruotato di 45 gradi
  const float k = 0.7071f;
  const float ax = k * l / 2, ay = k * l / 2, bx = k * w / 2, by = -k * w / 2;
  const int x0 = lroundf(cx - ax - bx), y0 = lroundf(cy - ay - by), x1 = lroundf(cx + ax - bx), y1 = lroundf(cy + ay - by);
  const int x2 = lroundf(cx + ax + bx), y2 = lroundf(cy + ay + by), x3 = lroundf(cx - ax + bx), y3 = lroundf(cy - ay + by);
  lcd.fillTriangle(x0, y0, x1, y1, x2, y2, c);
  lcd.fillTriangle(x0, y0, x2, y2, x3, y3, c);
}
static void drawLogIcon(int k, int x, int y, uint16_t c) {
  const uint16_t B = themeBg();
  switch (k) {
    case 0:   // orologio
      lcd.drawCircle(x, y, 6, c); lcd.drawCircle(x, y, 5, c);
      lcd.drawWideLine(x, y, x, y - 4, 0.8f, c); lcd.drawWideLine(x, y, x + 3, y + 1, 0.8f, c); break;
    case 1:   // percorso: puntina e puntini
      lcd.fillCircle(x + 3, y - 3, 4, c); lcd.fillTriangle(x - 1, y - 2, x + 7, y - 2, x + 3, y + 5, c);
      lcd.fillCircle(x + 3, y - 3, 1, B);
      lcd.fillCircle(x - 6, y + 6, 1, c); lcd.fillCircle(x - 3, y + 6, 1, c); lcd.fillCircle(x, y + 6, 1, c); break;
    case 2:   // pompa del carburante
      lcd.fillRoundRect(x - 6, y - 6, 8, 13, 1, c); lcd.fillRect(x - 4, y - 4, 4, 3, B);
      lcd.drawWideLine(x + 2, y - 4, x + 5, y - 2, 0.7f, c); lcd.drawWideLine(x + 5, y - 2, x + 5, y + 4, 0.7f, c);
      lcd.drawWideLine(x + 5, y + 4, x + 3, y + 4, 0.7f, c); break;
    case 3:   // aereo, prua a destra
      lcd.drawWideLine(x - 6, y, x + 6, y, 1.3f, c);
      lcd.fillTriangle(x + 2, y, x - 2, y - 7, x - 3, y, c); lcd.fillTriangle(x + 2, y, x - 2, y + 7, x - 3, y, c);
      lcd.fillTriangle(x - 4, y, x - 6, y - 3, x - 7, y, c); lcd.fillTriangle(x - 4, y, x - 6, y + 3, x - 7, y, c); break;
    case 4:   // montagne
      lcd.fillTriangle(x - 7, y + 6, x - 1, y - 5, x + 5, y + 6, c); lcd.fillTriangle(x - 1, y + 6, x + 3, y - 1, x + 7, y + 6, c); break;
    case 5:   // salita
      lcd.drawWideLine(x - 7, y + 5, x - 3, y + 1, 0.9f, c); lcd.drawWideLine(x - 3, y + 1, x, y + 3, 0.9f, c);
      lcd.drawWideLine(x, y + 3, x + 5, y - 3, 0.9f, c); lcd.fillTriangle(x + 7, y - 6, x + 1, y - 4, x + 6, y, c); break;
    case 6:   // termometro
      lcd.fillRoundRect(x - 2, y - 7, 5, 10, 2, c); lcd.fillCircle(x, y + 4, 3, c);
      lcd.drawFastHLine(x + 4, y - 5, 3, c); lcd.drawFastHLine(x + 4, y - 2, 3, c); lcd.drawFastHLine(x + 4, y + 1, 3, c); break;
    case 7:   // goccia
      lcd.fillCircle(x, y + 2, 4, c); lcd.fillTriangle(x - 4, y + 1, x + 4, y + 1, x, y - 7, c); break;
    case 8:   // satellite: corpo, due pannelli in diagonale, antenna e onde
      logRotRect(x + 1, y - 2, 4, 3.5f, c);
      logRotRect(x + 1 - 3.9f, y - 2 - 3.9f, 5, 5, c);
      logRotRect(x + 1 + 3.9f, y - 2 + 3.9f, 5, 5, c);
      lcd.drawLine(x - 1, y, x - 3, y + 2, c);
      lcd.fillArc(x + 1, y - 2, 6, 7, 100, 170, c);
      lcd.fillArc(x + 1, y - 2, 9, 10, 100, 170, c);
      break;
  }
}
// Righe del FLIGHT LOG: icona, voce e valore allineati a sinistra in colonna; ultima riga: satellite, orari e data.
static const int LOG_ICON_X = 53, LOG_LABEL_X = 65, LOG_VALUE_X = 221;
static void drawLogRows(const char *const *labels, char values[][32], int firstY, int stepY) {
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  for (int i = 0; i < 8; ++i) {
    const int y = firstY + i * stepY;
    drawLogIcon(i, LOG_ICON_X, y, themeColor(TFT_YELLOW));
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    lcd.setTextSize((1.1f) * THEME_TEXT_SCALE, (1.1f) * THEME_TEXT_SCALE);
    lcd.setTextDatum(ML_DATUM);
    setThemeTextColor(TFT_GREEN, TFT_BLACK);
    lcd.drawString(labels[i], LOG_LABEL_X, y);
    setThemeTextColor(TFT_WHITE, TFT_BLACK);
    lcd.drawString(values[i], LOG_VALUE_X, y);
  }
  const int yt = firstY + 8 * stepY;
  drawLogIcon(8, LOG_ICON_X, yt, themeColor(TFT_YELLOW));
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.05f) * THEME_TEXT_SCALE, (1.05f) * THEME_TEXT_SCALE);
  lcd.setTextDatum(ML_DATUM);
  setThemeTextColor(TFT_WHITE, TFT_BLACK);
  lcd.drawString(values[8], LOG_LABEL_X, yt);   // senza la scritta Time: orari e data completa
}

void drawFlightLogScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.20f) * THEME_TEXT_SCALE, (1.20f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  drawUiVlw("FLIGHT LOG", 180, 58, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);

  unsigned long elapsed = automaticFlightAccumMillis;
  if (automaticFlightRunning) elapsed += millis() - automaticFlightStartMillis;
  unsigned long sec = elapsed / 1000UL;
  // FUEL ESTIMATE (litri consumati stimati) al posto di Max Sink rate, subito dopo Km travelled
  const char* labels[9] = {
    "Flight Time", "Km travelled", "Approx Fuel", "Max GPS GS",
    "Max Altitude", "Max Climb rate", "Temp", "Humidity", "Time"
  };
  char values[9][32];
  snprintf(values[0], sizeof(values[0]), "%02lu:%02lu:%02lu", sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
  snprintf(values[1], sizeof(values[1]), "%.1f km", flightTotalDistanceKm);
  if (flightFuelUsedL >= 0.0f) snprintf(values[2], sizeof(values[2]), "%.1f l", flightFuelUsedL);
  else snprintf(values[2], sizeof(values[2]), "-- l");
  snprintf(values[3], sizeof(values[3]), "%.0f km/h", flightMaxGpsSpeedKmh);
  snprintf(values[4], sizeof(values[4]), "%.0f m", flightMaxAltitudeMeters);
  snprintf(values[5], sizeof(values[5]), "%.1f m/s", flightMaxClimbRate);
  float reportTemp = NAN;
  float reportHumidity = NAN;
  bool reportEnvironmentValid = false;
  bool reportNeedsCompensation = true;
  if (!automaticFlightRunning && flightEndEnvironmentValid) {
    reportTemp = flightEndTempC;
    reportHumidity = flightEndHumidity;
    reportEnvironmentValid = true;
    reportNeedsCompensation = false;
  } else if (sensorDataMutex != nullptr &&
             xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    reportTemp = meteo_temperature_c;
    reportHumidity = meteo_humidity_percent;
    reportEnvironmentValid = meteoValid && isfinite(reportTemp) && isfinite(reportHumidity) &&
                             reportHumidity >= 0.0f && reportHumidity <= 100.0f;
    xSemaphoreGive(sensorDataMutex);
  }
  if (reportEnvironmentValid) {
    float compensatedTemp = reportTemp;
    float compensatedHumidity = reportHumidity;
    if (reportNeedsCompensation) {
      compensateOutsideAirReading(reportTemp, reportHumidity, compensatedTemp, compensatedHumidity);
    }
    snprintf(values[6], sizeof(values[6]), "%.0fC", compensatedTemp);
    snprintf(values[7], sizeof(values[7]), "%.0f%%", compensatedHumidity);
  } else {
    snprintf(values[6], sizeof(values[6]), "--C");
    snprintf(values[7], sizeof(values[7]), "--%%");
  }
  char dateDisplay[12];
  if (strcmp(flightDate, "--/--/----") != 0) {
    strncpy(dateDisplay, flightDate, sizeof(dateDisplay)-1);
    dateDisplay[sizeof(dateDisplay)-1] = '\0';
  } else {
    strcpy(dateDisplay, "--/--/----");
  }
  snprintf(values[8], sizeof(values[8]), "%.5s/%.5s  %s", automaticStartUtc, automaticStopUtc, dateDisplay);
  drawLogRows(labels, values, 87, 26);

  const int savedLogsCenterX = SAVED_LOGS_BTN_X + SAVED_LOGS_BTN_W / 2;
  const int savedLogsCenterY = SAVED_LOGS_BTN_Y + SAVED_LOGS_BTN_H / 2;
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  const char* savedLogsText = "SAVED LOGS";
  lcd.drawRect(SAVED_LOGS_BTN_X, SAVED_LOGS_BTN_Y,
               SAVED_LOGS_BTN_W, SAVED_LOGS_BTN_H,
               themeColor(TFT_YELLOW));
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  lcd.drawString(savedLogsText, savedLogsCenterX, savedLogsCenterY);
}

void updateAutomaticFlightTimerScreen() {
  unsigned long elapsed = automaticFlightAccumMillis;
  if (automaticFlightRunning) elapsed += millis() - automaticFlightStartMillis;
  unsigned long sec = elapsed / 1000UL;
  char timer[16];
  snprintf(timer, sizeof(timer), "%02lu:%02lu:%02lu", sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
  lcd.setTextDatum(MC_DATUM);
  drawTimerGroup(timer, 150, automaticFlightRunning ? TFT_GREEN : TFT_RED,
                 1.25f * THEME_TEXT_SCALE);
}

void drawFlightLogsViewScreen(int index) {
  lcd.fillScreen(themeBg());
  FlightLog log;
  if (!readLogFromPrefs(index, log)) {
    drawNoLogsViewScreen();
    return;
  }
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.2f) * THEME_TEXT_SCALE, (1.2f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  char title[32];
  snprintf(title, sizeof(title), "LOGS");
  drawUiVlw(title, 180, 25, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);
  int total = getNumSavedLogs();

  const char* labels[9] = {
    "Flight Time", "Km travelled", "Approx Fuel", "Max GPS GS",
    "Max Altitude", "Max Climb rate", "Temp", "Humidity", "Time"
  };
  char values[9][32];
  unsigned long sec = log.durationSec;
  snprintf(values[0], sizeof(values[0]), "%02lu:%02lu:%02lu", sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
  snprintf(values[1], sizeof(values[1]), "%.1f km", log.distance);
  if (log.fuelUsed >= 0.0f && isfinite(log.fuelUsed)) snprintf(values[2], sizeof(values[2]), "%.1f l", log.fuelUsed);
  else snprintf(values[2], sizeof(values[2]), "-- l");   // log precedenti alla stima carburante
  snprintf(values[3], sizeof(values[3]), "%.0f km/h", log.maxSpeed);
  snprintf(values[4], sizeof(values[4]), "%.0f m", log.maxAltitude);
  snprintf(values[5], sizeof(values[5]), "%.1f m/s", log.maxClimb);
  if (log.endEnvironmentValid && isfinite(log.endTemp) && isfinite(log.endHumidity) &&
      log.endHumidity >= 0.0f && log.endHumidity <= 100.0f) {
    snprintf(values[6], sizeof(values[6]), "%.0fC", log.endTemp);
    snprintf(values[7], sizeof(values[7]), "%.0f%%", log.endHumidity);
  } else {
    snprintf(values[6], sizeof(values[6]), "--C");
    snprintf(values[7], sizeof(values[7]), "--%%");
  }
  snprintf(values[8], sizeof(values[8]), "%.5s/%.5s  %s", log.startTime, log.stopTime, log.date);
  drawLogRows(labels, values, 64, 25);   // un po' piu' in basso: la prima icona non tocca il bordo
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_YELLOW, TFT_BLACK);
  char pageText[16];
  snprintf(pageText, sizeof(pageText), "#%d/%d", index + 1, total);
  lcd.drawString(pageText, 110, 315);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("Tap to exit", 225, 315);
}

void drawNoLogsViewScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  lcd.setTextSize((1.2f) * THEME_TEXT_SCALE, (1.2f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_RED, TFT_BLACK);
  lcd.drawString("NO LOGS SAVED", 180, 180);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("clic to exit", 180, 310);
}

void drawFlightEndScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  lcd.setTextSize((1.35f) * THEME_TEXT_SCALE, (1.35f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_RED, TFT_BLACK);
  lcd.drawString("CLOSE", 180, 105);
  lcd.drawString("FLIGHT", 180, 155);
  lcd.drawString("PLAN", 180, 205);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  lcd.drawString("Click to continue", 180, 270);
}

// Versione del firmware dalla data di compilazione (__DATE__ = "Nov  5 2026"): "Version 26.11.05".
void firmwareVersionString(char *out, size_t n) {
  static const char *const MON = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *d = __DATE__;
  int month = 0;
  for (int m = 0; m < 12; ++m) if (strncmp(d, MON + m * 3, 3) == 0) { month = m + 1; break; }
  const int day = atoi(d + 4), year = atoi(d + 7);
  snprintf(out, n, "Version %02d.%02d.%02d", year % 100, month, day);
}
const char* firmwareSourceName() {
  const char* path = __FILE__;
  const char* slash = strrchr(path, '/');
  const char* backslash = strrchr(path, '\\');
  if (backslash != nullptr && (slash == nullptr || backslash > slash)) slash = backslash;
  return slash != nullptr ? slash + 1 : path;
}

static void resetPreferencesFromFirmware() {
  // Azzera le preferenze e i log (inclusi min/max G, salvati nei log voli).
  bool resetOk = true;
  if (preferences.begin("altimeter", false)) {
    if (!preferences.clear()) resetOk = false;
    // Valori di partenza espliciti: nvs_ver evita un secondo azzeramento al boot,
    // soglia vario = 1.0 m/s (indice 3), versioni profilo gia' aggiornate.
    preferences.putInt("nvs_ver", 1);
    preferences.putInt("av_threshold", 3);
    preferences.putInt("av_profile_ver", 2);
    preferences.putInt("vario_thr_ver", 1);
    preferences.end();
  } else {
    resetOk = false;
  }
  if (preferences.begin("display", false)) {
    if (!preferences.clear()) resetOk = false;
    preferences.end();
  } else {
    resetOk = false;
  }
  if (preferences.begin(FLIGHT_LOG_NS, false)) {
    if (!preferences.clear()) resetOk = false;
    preferences.end();
  } else {
    resetOk = false;
  }
  AVEA_LOGF("Reset preferences: %s\n", resetOk ? "OK" : "FALLITO (parziale)");

  lcd.fillScreen(themeBg());
  drawUiVlw(resetOk ? "Restarting..." : "Reset incomplete", 180, 180,
            MC_DATUM, themeColor(TFT_CYAN), themeBg(), 0.80f);
  // Riavvia comunque: le variabili in RAM (es. soglia vario) vengono cosi'
  // ricaricate dalla memoria e non restano ai vecchi valori.
  delay(resetOk ? 300 : 1200);
  ESP.restart();
}

static void drawFirmwareButton(const char *label, int centerX, int centerY,
                               int &boxX, int &boxY, int &boxW, int &boxH,
                               int padX, int padY) {
  const float scale = 0.62f;  // stessa UiFont32 e scala di "GO HOME"
  const bool loaded = vlwSet(lcd, UiFont32);
  if (loaded) {
    lcd.setTextSize(scale, scale);
  } else {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize(scale * 1.5f, scale * 1.5f);
  }
  lcd.setTextDatum(MC_DATUM);
  const uint16_t fg = firmwareButtonAlert ? themeColor(TFT_RED)
                                           : (displayTheme == THEME_COLOR ? TFT_CYAN : TFT_BLACK);
  const uint16_t bg = themeBg();
  lcd.setTextColor(fg, bg);
  const int textWidth = lcd.textWidth(label);
  int textHeight = lcd.fontHeight();
  if (textHeight <= 0) textHeight = 20;

  boxW = textWidth + 2 * padX;
  boxH = textHeight + 2 * padY;
  boxY = centerY - boxH / 2;
  {
    // Il display e' tondo: gli angoli in basso non devono uscire dal cerchio.
    const float dyBottom = (float)(boxY + boxH) - 180.0f;
    const float chord = 2.0f * sqrtf(fmaxf(0.0f, 180.0f * 180.0f - dyBottom * dyBottom)) - 8.0f;
    if ((float)boxW > chord) boxW = (int)chord;
    if (boxW < textWidth + 6) boxW = textWidth + 6;
  }
  boxX = centerX - boxW / 2;
  const uint16_t frame = firmwareButtonAlert ? themeColor(TFT_RED) : themeColor(TFT_CYAN);
  lcd.drawRect(boxX, boxY, boxW, boxH, frame);
  if (firmwareButtonAlert) lcd.drawRect(boxX + 1, boxY + 1, boxW - 2, boxH - 2, frame);   // bordo doppio
  lcd.drawString(label, centerX, centerY);
  if (loaded) vlwRelease(lcd);
}

static bool mountFirmwareSdCard() {
  if (!SD_MMC.setPins(TF_SD_CLK_PIN, TF_SD_CMD_PIN, TF_SD_D0_PIN)) {
    AVEA_LOGLN("SD: assegnazione pin SDMMC fallita");
    return false;
  }
  // FAT non viene mai formattata automaticamente se il mount fallisce.
  if (!SD_MMC.begin("/sdcard", true, false, 20000)) {
    AVEA_LOGLN("SD: mount fallito (scheda assente o filesystem non FAT)");
    return false;
  }
  const uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE) {
    AVEA_LOGLN("SD: nessuna scheda rilevata");
    SD_MMC.end();
    return false;
  }
  AVEA_LOGF("SD: pronta, capacita' %llu MB\n",
                (unsigned long long)(SD_MMC.cardSize() / (1024ULL * 1024ULL)));
  return true;
}

static void showFirmwareUpdateMessage(const char *message) {
  lcd.fillScreen(themeBg());
  drawUiVlw(message, 180, 180, MC_DATUM,
            themeColor(TFT_CYAN), themeBg(), 0.80f);
}

// Non piu' richiamato (pulsante sostituito da DEMO MODE): tenuto per un uso futuro, il compilatore non lo include.
[[maybe_unused]] static void updateFirmwareFromSd() {
  if (!sdCardReady) sdCardReady = mountFirmwareSdCard();
  if (!sdCardReady) {
    showFirmwareUpdateMessage("SD card not ready");
    delay(1200);
    drawFirmwareScreen();
    return;
  }

  File image = SD_MMC.open("/firmware.bin", FILE_READ);
  if (!image || image.isDirectory()) {
    if (image) image.close();
    showFirmwareUpdateMessage("firmware.bin not found");
    delay(1200);
    drawFirmwareScreen();
    return;
  }
  const size_t imageSize = image.size();
  const int imageMagic = image.read();
  if (imageSize == 0 || imageMagic != 0xE9) {
    image.close();
    AVEA_LOGLN("SD update: firmware.bin non e' un'immagine ESP valida (magic != 0xE9)");
    showFirmwareUpdateMessage("Invalid firmware.bin");
    delay(1500);
    drawFirmwareScreen();
    return;
  }
  image.seek(0);

  if (!Update.begin(imageSize, U_FLASH)) {
    const uint8_t updateError = Update.getError();
    AVEA_LOGF("SD update: Update.begin fallito: %s\n", Update.errorString());
    image.close();
    const char *message = updateError == UPDATE_ERROR_NO_PARTITION ? "OTA partition required" :
                          (updateError == UPDATE_ERROR_SPACE ? "Firmware too large" : "Update start failed");
    showFirmwareUpdateMessage(message);
    delay(1800);
    drawFirmwareScreen();
    return;
  }

  showFirmwareUpdateMessage("Updating firmware...");
  uint8_t buffer[2048];
  size_t totalWritten = 0;
  bool writeOk = true;
  while (image.available()) {
    const size_t bytesRead = image.read(buffer, sizeof(buffer));
    if (bytesRead == 0 || Update.write(buffer, bytesRead) != bytesRead) {
      writeOk = false;
      break;
    }
    totalWritten += bytesRead;
    esp_task_wdt_reset();
    delay(1);  // lascia eseguire gli altri task durante la scrittura
  }
  image.close();

  if (!writeOk || totalWritten != imageSize) {
    AVEA_LOGF("SD update: scrittura incompleta (%u/%u), errore %s\n",
                  (unsigned)totalWritten, (unsigned)imageSize, Update.errorString());
    Update.abort();
    showFirmwareUpdateMessage("Update write failed");
    delay(1500);
    drawFirmwareScreen();
    return;
  }
  if (!Update.end() || !Update.isFinished()) {
    AVEA_LOGF("SD update: verifica finale fallita: %s\n", Update.errorString());
    showFirmwareUpdateMessage("Update verify failed");
    delay(1500);
    drawFirmwareScreen();
    return;
  }

  AVEA_LOGLN("SD update: firmware verificato, riavvio");
  showFirmwareUpdateMessage("Update complete");
  delay(600);
  ESP.restart();
}

// ========== LICENZA: attivazione dello strumento ==========
// Ogni strumento ha un codice di attivazione di 8 caratteri (0-9, A-F) calcolato dal suo MAC
// (quello mostrato nella pagina FIRMWARE VERSION) con una chiave segreta: HMAC-SHA256, primi 4 byte.
// Senza codice valido lo strumento funziona SOLO in DEMO (la demo non si puo' fermare).
// Il codice e' salvato nel namespace NVS "lic": "Reset preferences" non lo cancella.
// La chiave e' scritta mascherata, cosi' non si legge come testo dentro il file .bin.
// ATTENZIONE: questo sketch contiene la chiave segreta. Pubblicare su GitHub solo il .bin, mai sketch.ino.
static const uint8_t LIC_KEY_MASKED[24] = { 0x71, 0x46, 0x68, 0x7D, 0x55, 0xD4, 0x61, 0x9E, 0x51, 0x24, 0xCD, 0x2B, 0xAD, 0x17, 0xF6, 0x34, 0x9E, 0x83, 0x0B, 0x88, 0x73, 0xA0, 0xA2, 0x83 };
bool licenseValid = false;
static char licenseEntry[9] = "";
static ScreenState licenseReturnScreen = FIRMWARE_SCREEN;
static bool licenseWrongCode = false;
static const int LIC_KEY_X0 = 90, LIC_KEY_W = 45, LIC_KEY_Y0 = 136, LIC_KEY_H = 38;   // tastierino 4 x 4
static const char LIC_KEYS[] = "0123456789ABCDEF";
static const int LIC_BTN_Y = 296, LIC_BTN_H = 34;                                    // DEL / OK

// Codice atteso per questo strumento (8 caratteri + terminatore)
static bool licenseExpectedCode(char *out9) {
  char mac[18];
  otaGetMac(mac);
  uint8_t key[sizeof(LIC_KEY_MASKED)];
  for (size_t i = 0; i < sizeof(key); ++i) key[i] = LIC_KEY_MASKED[i] ^ (uint8_t)((i * 29 + 0x5A) & 0xFF);
  uint8_t h[32];
  const int rc = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), key, sizeof(key),
                                 (const unsigned char *)mac, strlen(mac), h);
  memset(key, 0, sizeof(key));
  if (rc != 0) { out9[0] = '\0'; return false; }
  snprintf(out9, 9, "%02X%02X%02X%02X", h[0], h[1], h[2], h[3]);
  return true;
}

static bool licenseCheckCode(const char *code) {
  char expected[9];
  if (code == nullptr || strlen(code) != 8 || !licenseExpectedCode(expected)) return false;
  return strncmp(code, expected, 8) == 0;   // il tastierino scrive solo maiuscole, come il codice atteso
}

void loadLicense() {
  char code[12] = "";
  if (preferences.begin("lic", true)) {
    preferences.getString("code", code, sizeof(code));
    preferences.end();
  }
  licenseValid = licenseCheckCode(code);
  AVEA_LOGF("LICENZA: %s\n", licenseValid ? "strumento attivato" : "NON attivato (solo DEMO)");
}

static void licenseDrawEntry() {
  const uint16_t bg = themeBg();
  lcd.fillRect(60, 80, 240, 50, bg);
  const uint16_t frame = licenseWrongCode ? themeVivid(TFT_RED) : themeVivid(TFT_YELLOW);
  lcd.drawRoundRect(78, 86, 204, 40, 8, frame);
  lcd.drawRoundRect(79, 87, 202, 38, 7, frame);
  char shown[10];
  for (int i = 0, j = 0; i < 8; ++i) {
    shown[j++] = (i < (int)strlen(licenseEntry)) ? licenseEntry[i] : '_';
    if (i == 3) shown[j++] = ' ';
    shown[j] = '\0';
  }
  drawUiVlw(licenseWrongCode ? "WRONG CODE" : shown, 180, 106, MC_DATUM,
            licenseWrongCode ? themeVivid(TFT_RED) : themeColor(TFT_WHITE), bg, 0.85f);
}

void drawLicenseScreen() {
  const uint16_t bg = themeBg();
  lcd.fillScreen(bg);
  drawUiVlw("ACTIVATION", 180, 36, MC_DATUM, themeColor(TFT_CYAN), bg, 0.8f);
  char mac[18], line[28];
  otaGetMac(mac);
  snprintf(line, sizeof(line), "MAC %s", mac);
  drawUiVlw(line, 180, 64, MC_DATUM, themeColor(TFT_WHITE), bg, 0.5f);
  licenseDrawEntry();
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      const int x = LIC_KEY_X0 + c * LIC_KEY_W, y = LIC_KEY_Y0 + r * LIC_KEY_H;
      lcd.drawRoundRect(x + 2, y + 2, LIC_KEY_W - 4, LIC_KEY_H - 4, 6, themeColor(TFT_DARKGREY));
      const char key[2] = { LIC_KEYS[r * 4 + c], '\0' };
      drawUiVlw(key, x + LIC_KEY_W / 2, y + LIC_KEY_H / 2, MC_DATUM, themeColor(TFT_WHITE), bg, 0.6f);
    }
  }
  lcd.drawRoundRect(92, LIC_BTN_Y, 80, LIC_BTN_H, 8, themeColor(TFT_ORANGE));
  lcd.drawRoundRect(93, LIC_BTN_Y + 1, 78, LIC_BTN_H - 2, 7, themeColor(TFT_ORANGE));
  drawUiVlw("DEL", 132, LIC_BTN_Y + LIC_BTN_H / 2, MC_DATUM, themeColor(TFT_ORANGE), bg, 0.6f);
  lcd.drawRoundRect(188, LIC_BTN_Y, 80, LIC_BTN_H, 8, themeVivid(TFT_GREEN));
  lcd.drawRoundRect(189, LIC_BTN_Y + 1, 78, LIC_BTN_H - 2, 7, themeVivid(TFT_GREEN));
  drawUiVlw("OK", 228, LIC_BTN_Y + LIC_BTN_H / 2, MC_DATUM, themeVivid(TFT_GREEN), bg, 0.6f);
}

// Apre la pagina di attivazione; returnScreen = pagina a cui tornare (int: vedi prototipi generati da Arduino)
void openLicenseScreen(int returnScreen) {
  licenseReturnScreen = (ScreenState)returnScreen;
  licenseWrongCode = false;
  licenseEntry[0] = '\0';
  currentScreen = LICENSE_SCREEN;
  drawLicenseScreen();
}

static void licenseExit() {
  currentScreen = licenseReturnScreen;
  for (int i = 0; i < numScreens; ++i) {
    if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
  }
  lastTouchMillis = millis();
  redrawCurrentScreen();
}

// Rilascio del dito sulla pagina di attivazione: tastierino, DEL, OK; swipe dx/sx = esce (lo strumento resta in DEMO)
void licenseTouchRelease(bool tapValid, int diffX, int diffY, int tx, int ty) {
  if (!tapValid) {
    if (abs(diffX) > SWIPE_MIN_X_PX && abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX) {
      playTone(1500, 80);
      licenseExit();
    }
    return;
  }
  if (tx >= LIC_KEY_X0 && tx < LIC_KEY_X0 + 4 * LIC_KEY_W &&
      ty >= LIC_KEY_Y0 && ty < LIC_KEY_Y0 + 4 * LIC_KEY_H) {
    const int c = (tx - LIC_KEY_X0) / LIC_KEY_W, r = (ty - LIC_KEY_Y0) / LIC_KEY_H;
    const size_t len = strlen(licenseEntry);
    if (len >= 8) { playTone(600, 60); return; }   // codice gia' completo
    licenseEntry[len] = LIC_KEYS[r * 4 + c];
    licenseEntry[len + 1] = '\0';
    licenseWrongCode = false;
    licenseDrawEntry();
    playTone(1200, 20);
    return;
  }
  if (ty < LIC_BTN_Y - 10 || ty > LIC_BTN_Y + LIC_BTN_H + 14) return;
  if (tx >= 80 && tx < 180) {          // DEL
    const size_t len = strlen(licenseEntry);
    if (len > 0) licenseEntry[len - 1] = '\0';
    licenseWrongCode = false;
    licenseDrawEntry();
    playTone(1200, 30);
  } else if (tx >= 180 && tx <= 280) { // OK
    if (licenseCheckCode(licenseEntry)) {
      if (preferences.begin("lic", false)) {
        preferences.putString("code", licenseEntry);
        preferences.end();
      }
      licenseValid = true;
      licenseEntry[0] = '\0';
      AVEA_LOGLN("LICENZA: codice corretto, strumento attivato");
      if (demoMode) {
        stopDemoMode(millis());
        if (bleRole == BLE_ROLE_SLAVE) bleQueueDemo(false);
      }
      playTone(1500, 100);
      playTone(2000, 150);
      licenseExit();
    } else {
      licenseWrongCode = true;
      licenseEntry[0] = '\0';
      licenseDrawEntry();
      playTone(600, 200);
    }
  }
}

// Indirizzo del manuale (codice QR della pagina FIRMWARE VERSION)
static const char *const MANUAL_QR_URL = "https://drive.google.com/drive/u/0/folders/1vRXCJjZf-rTIVcHebrR4INtRvPpX6Bbm";

void drawFirmwareScreen() {
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
  lcd.setTextSize((1.5f) * THEME_TEXT_SCALE, (1.5f) * THEME_TEXT_SCALE);
  setThemeTextColor(TFT_CYAN, TFT_BLACK);
  drawUiVlw("FIRMWARE", 180, 52, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);
  drawUiVlw("VERSION", 180, 84, MC_DATUM, themeColor(TFT_CYAN), themeBg(), THEME_TEXT_SCALE);
  // "MAC ..." e la versione ricavata dalla data di compilazione (anno.mese.giorno, es. Version 26.11.05).
  // Strumento non attivato: in mezzo la scritta rossa DEMO - NOT ACTIVATED.
  {
    char macStr[18], macLine[28];
    otaGetMac(macStr);
    snprintf(macLine, sizeof(macLine), "MAC %s", macStr);
    char verLine[24];
    firmwareVersionString(verLine, sizeof(verLine));
    if (licenseValid) {
      drawUiVlw(macLine, 180, 112, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.6f);
      drawUiVlw(verLine, 180, 134, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.6f);
    } else {
      drawUiVlw(macLine, 180, 110, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.6f);
      drawUiVlw("DEMO - NOT ACTIVATED", 180, 128, MC_DATUM, themeVivid(TFT_RED), themeBg(), 0.6f);
      drawUiVlw(verLine, 180, 146, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.6f);
    }
  }
  // Codice QR: inquadrandolo con il telefono si apre il repository con il manuale
  lcd.fillRect(140, 155, 80, 80, TFT_WHITE);                  // fondo bianco con margine (zona di quiete)
  lcd.qrcode(MANUAL_QR_URL, 142, 157, 76, 3);
  // A sinistra del QR, centrate tra il bordo e il codice e in verticale sul codice:
  // "Download guide and AVEA link" spezzato su tre righe corte, per starci nella fascia
  // che resta fra il bordo del tondo e il codice.
  drawUiVlw("Download", 78, 173, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.6f);
  drawUiVlw("guide and", 78, 195, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.6f);
  drawUiVlw("AVEA link", 78, 217, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.6f);
  // A destra del QR, in simmetria: "TAP TO" e sotto "PAGES" (tocco = PIN, poi la pagina PAGES)
  drawUiVlw("TAP TO", 282, 184, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.6f);
  drawUiVlw("PAGES", 282, 206, MC_DATUM, themeColor(TFT_YELLOW), themeBg(), 0.6f);
  // Pulsante DEMO MODE (al posto di "Update from SD"): un clic avvia la demo, con la demo attiva la ferma
  if (firmwareResetArmedMs == 0) {
    // Strumento non attivato: al posto di DEMO MODE il pulsante ACTIVATE (apre l'inserimento del codice)
    drawFirmwareButton(!licenseValid ? "ACTIVATE" : (demoMode ? "STOP DEMO" : "DEMO MODE"), FIRMWARE_UPDATE_BTN_CENTER_X,
                       FIRMWARE_UPDATE_BTN_CENTER_Y, firmwareUpdateButtonX,
                       firmwareUpdateButtonY, firmwareUpdateButtonW, firmwareUpdateButtonH,
                       FIRMWARE_BUTTON_PADDING, FIRMWARE_BUTTON_PADDING);
  } else {
    // Reset armato: al posto di DEMO MODE l'avviso di cosa fa il secondo clic
    drawUiVlw("Settings and logs", 180, 243, MC_DATUM, themeColor(TFT_RED), themeBg(), 0.55f);
    drawUiVlw("will be erased", 180, 263, MC_DATUM, themeColor(TFT_RED), themeBg(), 0.55f);
  }
  const bool resetArmed = firmwareResetArmedMs != 0;
  firmwareButtonAlert = resetArmed;
  drawFirmwareButton(resetArmed ? "CONFIRM RESET?" : "Reset preferences", FIRMWARE_RESET_BTN_CENTER_X,
                     FIRMWARE_RESET_BTN_CENTER_Y, firmwareResetButtonX,
                     firmwareResetButtonY, firmwareResetButtonW, firmwareResetButtonH,
                     FIRMWARE_RESET_PAD_X, FIRMWARE_RESET_PAD_Y);
  firmwareButtonAlert = false;
}

// Pagina nascosta PAGES: due colonne (7 + 6 righe) con una casella per ogni pagina del giro.
static const int PAGE_SEL_COL_X[2] = {48, 188};
static const int PAGE_SEL_ROW_Y0 = 66;
static const int PAGE_SEL_ROW_H = 34;
static const int PAGE_SEL_ROWS = 7;

int pageSelectSubPage = 0;   // 0 = PAGES 1/3, 1 = PAGES 2/3, 2 = PAGES 3/3 (clic sul titolo per passare alla successiva)
static const int PAGE_SEL2_X = 62, PAGE_SEL2_Y0 = 62, PAGE_SEL2_ROW_H = 32;   // 8 righe (con AUDIO)
static const int PAGES2_COUNT = 5;   // PAGES 2/2: MANUAL FLIGHT TIMER, ROTATE DISPLAY (G-METER), FUEL ESTIMATE, NEAREST FIELDS, FIELDS 30 KM
static const int PAGE_SEL2_ROWS = PAGES2_COUNT + 3;   // pagine + COMPASS VERTICAL SWIPE + ARCH BLOCK + ICT / ONLY CTR
static const int PAGE_SEL2_CTR_X = 170;   // casella ONLY CTR a destra di ICT
static const int PAGE_SEL2_SET_X = 262;   // pulsante SET sulla riga FUEL ESTIMATE
static const int PAGE_SEL2_GS150_X = 222;  // casella GS 150 a destra di ARCH BLOCK
static const int HR_BTN_X = 90, HR_BTN_Y = 160, HR_BTN_W = 180, HR_BTN_H = 56;   // PAGES 3/3: pulsante HARD RESET (sotto DON'T SHOW ALERTS)
static const int SLIP_BOX_X = 62, SLIP_BOX_Y = 62;     // PAGES 3/3: casella TURN & SLIP in alto a sinistra, come la prima riga di PAGES 2/3
static const int NOALERT_BOX_X = 62, NOALERT_BOX_Y = 92;   // PAGES 3/3: casella DON'T SHOW ALERTS, sotto TURN & SLIP

// PAGES: tastiera del PIN (vedi drawPinPad) mostrata dentro PAGE_SELECT_SCREEN finche' il PIN non e' giusto.
static bool pagesPinMode = false;      // true = mostra la tastiera del PIN invece di PAGES
static bool hardResetPinMode = false;   // PAGES 3/3: tastiera della password di HARD RESET
static const char *const HARD_RESET_PIN = "1965";
static void openPagesPin() {
  pinPadReset();
  pinPadTitle = "PIN";
  hardResetPinMode = false;
  pagesPinMode = true;
  currentScreen = PAGE_SELECT_SCREEN;
  pageSelectSubPage = 0;
  drawPinPad();
}
void drawPageSelectScreen();
static void pagesPinTap(int tx, int ty) {
  if (pinPadTap(tx, ty)) { pagesPinMode = false; drawPageSelectScreen(); }
}
void drawFirmwareScreen();
// HARD RESET (PAGES 3/3): con la password cancella il codice di attivazione. Lo strumento torna in DEMO
// e il codice va reinserito (ACTIVATE in FIRMWARE VERSION), come al primo acquisto.
static void hardResetPinTap(int tx, int ty) {
  if (!pinPadTap(tx, ty, HARD_RESET_PIN)) return;
  hardResetPinMode = false;
  pinPadTitle = "PIN";
  if (preferences.begin("lic", false)) { preferences.remove("code"); preferences.end(); }
  licenseValid = false;
  AVEA_LOGLN("HARD RESET: codice di attivazione cancellato");
  lcd.fillScreen(themeBg());
  drawUiVlw("ACTIVATION", 180, 150, MC_DATUM, themeVivid(TFT_RED), themeBg(), 0.85f);
  drawUiVlw("CODE ERASED", 180, 190, MC_DATUM, themeVivid(TFT_RED), themeBg(), 0.85f);
  playTone(900, 300);
  delay(1500);
  currentScreen = FIRMWARE_SCREEN;   // da qui ACTIVATE per reinserire il codice
  drawFirmwareScreen();
}

static int screenIndexOf(int sc);
static int pageSel1Index(int pos);
void drawPageSelectScreen() {
  if (pagesPinMode || hardResetPinMode) { drawPinPad(); return; }
  lcd.fillScreen(themeBg());
  if (pageSelectSubPage > 2) pageSelectSubPage = 2;
  drawUiVlw(pageSelectSubPage == 0 ? "PAGES 1/3" : (pageSelectSubPage == 1 ? "PAGES 2/3" : "PAGES 3/3"), 180, 34, MC_DATUM,
            themeColor(TFT_CYAN), themeBg(), 0.9f * THEME_TEXT_SCALE);
  if (pageSelectSubPage == 2) {   // PAGES 3/3: HARD RESET (cancella il codice di attivazione, con password)
    const uint16_t red = themeVivid(TFT_RED);
    lcd.drawRoundRect(HR_BTN_X, HR_BTN_Y, HR_BTN_W, HR_BTN_H, 10, red);
    lcd.drawRoundRect(HR_BTN_X + 1, HR_BTN_Y + 1, HR_BTN_W - 2, HR_BTN_H - 2, 9, red);
    drawUiVlw("HARD RESET", 180, HR_BTN_Y + HR_BTN_H / 2 + 1, MC_DATUM, red, themeBg(), 0.7f);
    drawUiVlw("erases the activation code", 180, HR_BTN_Y + HR_BTN_H + 26, MC_DATUM, themeColor(TFT_LIGHTGREY), themeBg(), 0.45f);
    drawUiVlw("(password required)", 180, HR_BTN_Y + HR_BTN_H + 48, MC_DATUM, themeColor(TFT_LIGHTGREY), themeBg(), 0.45f);
    {   // casella TURN & SLIP (sketch_test)
      const int si = screenIndexOf(SLIP_SCREEN);
      const bool on = si >= 0 && pageIndexEnabled(si);
      const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
      const uint16_t txtColor = on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY);
      const int x = SLIP_BOX_X, y = SLIP_BOX_Y;
      lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
      lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
      if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
      const int tw = drawUiVlw("TURN & SLIP", x + 28, y + 15, ML_DATUM, txtColor, themeBg(), 0.5f);   // come le righe di PAGES 2/3
      if (!slipSensorFound()) drawUiVlw("(no sensor)", x + 28 + tw + 8, y + 15, ML_DATUM, themeColor(TFT_DARKGREY), themeBg(), 0.4f);
    }
    {   // casella DON'T SHOW ALERTS: nessun avviso a tutto schermo
      const bool on = alertsMuted;
      const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
      const uint16_t txtColor = on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY);
      const int x = NOALERT_BOX_X, y = NOALERT_BOX_Y;
      lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
      lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
      if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
      drawUiVlw("DON'T SHOW ALERTS", x + 28, y + 15, ML_DATUM, txtColor, themeBg(), 0.5f);
    }
    drawUiVlw("Swipe to exit", 180, 340, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.5f);
    return;
  }
  if (pageSelectSubPage == 1) {
    // PAGES 2/2: MANUAL FLIGHT TIMER, ROTATE DISPLAY 180 (riga di G-METER), FUEL ESTIMATE, ...
    for (int k = 0; k < PAGES2_COUNT; ++k) {
      const int i = pageSel2Index(k);
      const int x = PAGE_SEL2_X;
      const int y = PAGE_SEL2_Y0 + k * PAGE_SEL2_ROW_H;
      if (screenOrder[i] == G_METER_SCREEN) {   // al posto di G-METER (TEST): ROTATE DISPLAY 180 gradi
        const bool on = displayRotated180;
        const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
        const uint16_t txtColor = on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY);
        lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
        lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
        if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
        const int w = drawUiVlw("ROTATE DISPLAY 180", x + 28, y + 15, ML_DATUM, txtColor, themeBg(), 0.5f);
        lcd.drawCircle(x + 28 + w + 4, y + 7, 3, txtColor);   // simbolo dei gradi
        lcd.drawCircle(x + 28 + w + 4, y + 7, 2, txtColor);
        continue;
      }
      const bool on = pageIndexEnabled(i);
      const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
      lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
      lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
      if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
      drawUiVlw(PAGE_NAMES[i], x + 28, y + 15, ML_DATUM,
                              on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
      if (screenOrder[i] == FUEL_SCREEN) {   // SET: impostazioni carburante, sempre raggiungibili
        lcd.drawRoundRect(PAGE_SEL2_SET_X, y + 2, 56, 28, 6, themeColor(TFT_ORANGE));
        lcd.drawRoundRect(PAGE_SEL2_SET_X + 1, y + 3, 54, 26, 5, themeColor(TFT_ORANGE));
        drawUiVlw("SET", PAGE_SEL2_SET_X + 28, y + 16, MC_DATUM, themeColor(TFT_ORANGE), themeBg(), 0.5f);
      }
    }
    {   // riga COMPASS VERTICAL SWIPE (non e' una pagina)
      const int x = PAGE_SEL2_X;
      const int y = PAGE_SEL2_Y0 + (PAGE_SEL2_ROWS - 3) * PAGE_SEL2_ROW_H;
      const bool on = !verticalSwipeDisabled;   // COMPASS VERTICAL SWIPE: spuntato = swipe attivo sulla bussola
      const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
      lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
      lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
      if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
      drawUiVlw("COMPASS VERTICAL SWIPE", x + 28, y + 15, ML_DATUM,
                on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
    }
    {   // penultima riga: ARCH BLOCK (blocca lo spostamento delle soglie sugli archi)
      const int x = PAGE_SEL2_X;
      const int y = PAGE_SEL2_Y0 + (PAGE_SEL2_ROWS - 2) * PAGE_SEL2_ROW_H;
      const bool on = arcBlockEnabled;
      const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
      lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
      lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
      if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
      drawUiVlw("ARCH BLOCK", x + 28, y + 15, ML_DATUM,
                on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
      // a destra sulla stessa riga: GS 150 (fondo scala GS da 0-150 km/h)
      const bool on150 = gsScale150;
      const uint16_t box150 = on150 ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
      const int x2 = PAGE_SEL2_GS150_X;
      lcd.drawRoundRect(x2, y + 4, 22, 22, 4, box150);
      lcd.drawRoundRect(x2 + 1, y + 5, 20, 20, 3, box150);
      if (on150) lcd.fillRoundRect(x2 + 5, y + 9, 12, 12, 2, box150);
      drawUiVlw("GS 150", x2 + 28, y + 15, ML_DATUM,
                on150 ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
    }
    {   // ultima riga: ICT (pagina on / off, spostata qui da PAGES 1/3) e a destra ONLY CTR
      const int y = PAGE_SEL2_Y0 + (PAGE_SEL2_ROWS - 1) * PAGE_SEL2_ROW_H;
      auto box = [&](int x, bool on, const char *label) {
        const uint16_t c = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
        lcd.drawRoundRect(x, y + 4, 22, 22, 4, c);
        lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, c);
        if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, c);
        drawUiVlw(label, x + 28, y + 15, ML_DATUM, on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
      };
      const int ictIdx = screenIndexOf(ICT_SCREEN);
      box(PAGE_SEL2_X, ictIdx >= 0 && pageIndexEnabled(ictIdx), "ICT");
      box(PAGE_SEL2_CTR_X, aspOnlyCtr, "ONLY CTR");
    }
    drawUiVlw("Swipe to exit", 180, 340, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.5f);
    return;
  }
  for (int pos = 0; pos < PAGE_SEL_ROWS * 2 - 1; ++pos) {   // l'ultima casella della 2a colonna e' OFFSET
    const int i = pageSel1Index(pos);   // SETUP nascosta: le pagine dopo salgono di una riga
    if (i == -2) continue;               // casella occupata da OFFSET (METEO OFFSET)
    if (i < 0) break;
    const int col = pos / PAGE_SEL_ROWS;
    const int row = pos % PAGE_SEL_ROWS;
    const int x = PAGE_SEL_COL_X[col];
    const int y = PAGE_SEL_ROW_Y0 + row * PAGE_SEL_ROW_H;
    const bool on = pageIndexEnabled(i);
    const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
    lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
    lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
    if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
    drawUiVlw(PAGE_NAMES[i], x + 28, y + 15, ML_DATUM,
              on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
  }
  // Voce OFFSET (METEO OFFSET) nella seconda colonna, al posto di ICT (spostata in PAGES 3/3)
  {
    const int x = PAGE_SEL_COL_X[1], y = PAGE_SEL_ROW_Y0 + 3 * PAGE_SEL_ROW_H;
    lcd.drawRoundRect(x - 4, y + 2, 124, 28, 6, themeColor(TFT_ORANGE));
    lcd.drawRoundRect(x - 3, y + 3, 122, 26, 5, themeColor(TFT_ORANGE));
    drawUiVlw("OFFSET", x + 58, y + 16, MC_DATUM, themeColor(TFT_ORANGE), themeBg(), 0.5f);
  }
  // Voce ALT OFFSET nella seconda colonna, sulla riga di ALTIMETER: apre la taratura della quota dell'altimetro
  {
    const int x = PAGE_SEL_COL_X[1], y = PAGE_SEL_ROW_Y0 + (PAGE_SEL_ROWS - 2) * PAGE_SEL_ROW_H;   // riga di ALTIMETER
    lcd.drawRoundRect(x - 4, y + 2, 124, 28, 6, themeColor(TFT_ORANGE));
    lcd.drawRoundRect(x - 3, y + 3, 122, 26, 5, themeColor(TFT_ORANGE));
    drawUiVlw("ALT OFFSET", x + 58, y + 16, MC_DATUM, themeColor(TFT_ORANGE), themeBg(), 0.5f);
  }
  // Casella LUBBER LINE nella seconda colonna, ultima riga: spuntata = impostazioni del corridoio apribili da GO TO HOME
  {
    const int x = PAGE_SEL_COL_X[1], y = PAGE_SEL_ROW_Y0 + (PAGE_SEL_ROWS - 1) * PAGE_SEL_ROW_H;
    const bool on = lubberEditEnabled;
    const uint16_t boxColor = on ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
    lcd.drawRoundRect(x, y + 4, 22, 22, 4, boxColor);
    lcd.drawRoundRect(x + 1, y + 5, 20, 20, 3, boxColor);
    if (on) lcd.fillRoundRect(x + 5, y + 9, 12, 12, 2, boxColor);
    drawUiVlw("LUBBER LINE", x + 28, y + 15, ML_DATUM,
              on ? themeColor(TFT_WHITE) : themeColor(TFT_DARKGREY), themeBg(), 0.5f);
  }
  drawUiVlw("Swipe to exit", 180, 328, MC_DATUM, themeColor(TFT_WHITE), themeBg(), 0.5f);
}

// PAGES 1/2: ordine delle caselle sul display (indici di screenOrder). SETUP e' nascosta e non occupa righe,
// cosi' WIFI LOG e GPS INFO stanno subito sotto ICT. La maschera salvata resta per indice di screenOrder.
static int screenIndexOf(int sc) {
  for (int i = 0; i < numScreens; ++i) if ((int)screenOrder[i] == sc) return i;
  return -1;
}
static int pageSel1Index(int pos) {
  // Disposizione delle caselle (solo sul display): 1a colonna COMPASS, GO HOME, GPS INFO, METEO, QFE,
  // ALTIMETER, VARIO; 2a colonna GPS SPEED, GPS CLOCK, AUTO TIMER, ICT, WIFI LOG, (vuota), OFFSET.
  static const ScreenState PAGE1_LAYOUT[] = {
    COMPASS_SCREEN, GO_TO_HOME_SCREEN, GPS_INFO_SCREEN, METEO_SCREEN, QFE_SCREEN, MAIN_ALT_SCREEN, VARIOMETER_SCREEN,
    GPS_SPEED_SCREEN, GPS_CLOCK_SCREEN, AUTOMATIC_TIMER_SCREEN, ICT_SCREEN, WIFI_LOGS_SCREEN
  };
  if (pos < 0 || pos >= (int)(sizeof(PAGE1_LAYOUT) / sizeof(PAGE1_LAYOUT[0]))) return -1;
  if (PAGE1_LAYOUT[pos] == ICT_SCREEN) return -2;   // al posto di ICT c'e' OFFSET (ICT in PAGES 3/3)
  return screenIndexOf(PAGE1_LAYOUT[pos]);
}
// PAGES 2/2: riga k -> indice di screenOrder. Al posto di FIRMWARE VERSION (sempre attiva) c'e' NEAREST FIELDS.
static int pageSel2Index(int k) {
  if (k < 0 || k >= PAGES2_COUNT) return -1;
  if (k == 4) return screenIndexOf(FIELDS_RADAR_SCREEN);   // quinta riga: FIELDS 30 KM
  if (screenOrder[PAGES_FIRST_COUNT + k] == FIRMWARE_SCREEN) return screenIndexOf(NEAREST_FIELDS_SCREEN);
  return PAGES_FIRST_COUNT + k;
}
// Indice della pagina sotto il tocco (-1 se nessuna).
static int pageSelectHitTest(int tx, int ty) {
  if (tx < 30 || tx > 340) return -1;
  if (ty < PAGE_SEL_ROW_Y0 || ty >= PAGE_SEL_ROW_Y0 + PAGE_SEL_ROWS * PAGE_SEL_ROW_H) return -1;
  const int col = tx < PAGE_SEL_COL_X[1] - 4 ? 0 : 1;
  const int row = (ty - PAGE_SEL_ROW_Y0) / PAGE_SEL_ROW_H;
  return pageSel1Index(col * PAGE_SEL_ROWS + row);
}
// PAGES 2/2: indice della pagina sotto il tocco (-1 se nessuna).
static int pageSelect2HitTest(int tx, int ty) {
  if (tx < 40 || tx > 330) return -1;
  for (int k = 0; k < PAGE_SEL2_ROWS; ++k) {
    const int y = PAGE_SEL2_Y0 + k * PAGE_SEL2_ROW_H;
    if (ty >= y - (k == 0 ? 6 : 0) && ty < y + PAGE_SEL2_ROW_H + (k == PAGE_SEL2_ROWS - 1 ? 8 : 0))
      return k < PAGES2_COUNT ? pageSel2Index(k) : numScreens + (k - PAGES2_COUNT);   // numScreens = VERTICAL SWIPE, +1 = ARCH BLOCK, +2 = ICT / ONLY CTR
  }
  return -1;
}

// Tocco sulla scritta A.V.E.A* della bussola: logo per 5 s, poi di nuovo la bussola (un tocco torna subito).
static bool compassLogoActive = false;
static unsigned long compassLogoUntil = 0;
void showCompassLogo() {
  compassLogoActive = true;
  compassLogoUntil = millis() + 5000UL;
  currentScreen = AVEA_LOGO_SCREEN;
  drawAveaLogoScreen();
}
void endCompassLogo() {
  if (!compassLogoActive) return;
  compassLogoActive = false;
  if (currentScreen == AVEA_LOGO_SCREEN) {
    currentScreen = COMPASS_SCREEN;
    drawCompassScreen(true);
  }
}
void updateCompassLogo() {
  if (!compassLogoActive) return;
  if (currentScreen != AVEA_LOGO_SCREEN) { compassLogoActive = false; return; }   // interrotto da un avviso
  if ((long)(millis() - compassLogoUntil) >= 0) endCompassLogo();
}

void drawAveaLogoScreen() {
  const int displaySize = 360;
  // Zoom centrale lieve: rimuove parte dei margini del bitmap per sfruttare
  // meglio il display circolare mantenendo quasi tutta la cornice visibile.
  const int logoLeft = 12;
  const int logoRight = 347;
  const int logoTop = 12;
  const int logoBottom = 347;
  const uint16_t *logoPixels = reinterpret_cast<const uint16_t*>(img_logo_1__data);
  uint16_t rowBuffer[displaySize];

  // Riutilizza il bitmap A.V.E.A. con lo stesso zoom centrale della schermata
  // di avvio, per occupare meglio il pannello quadrato.
  for (int y = 0; y < displaySize; ++y) {
    const int sourceY = logoTop + y * (logoBottom - logoTop) / (displaySize - 1);
    for (int x = 0; x < displaySize; ++x) {
      const int sourceX = logoLeft + x * (logoRight - logoLeft) / (displaySize - 1);
      rowBuffer[x] = logoPixels[sourceY * displaySize + sourceX];
    }
    lcd.pushImage(0, y, displaySize, 1, rowBuffer);
  }
}

void playStartupMusic() {
  if (!isAudioEnabled) return;
  playTone(262, 100); delay(10);
  playTone(330, 100); delay(10);
  playTone(392, 100); delay(10);
  playTone(523, 150);
}

// ===================== TURN & SLIP (solo sketch_test) =====================
// Sensore esterno sullo stesso bus I2C del BME280:
//  - MPU-6050 (0x68 con AD0 a GND, 0x69 con AD0 a 3,3 V): accelerometro per la pallina e GIROSCOPIO per
//    l'aereo (rateo di virata immediato, come un coordinatore di virata vero);
// Montaggio come prima: scheda incollata dietro al display, componenti verso il muso. Con l'MPU-6050 la
// rotazione della scheda nel suo piano non conta: la verticale si ricava dalla gravita' (a terra all'accensione,
// o con SET 0 in volo dritto e livellato). Se la scheda guarda la coda: MPU_FACES_NOSE = false.
class SlipBall {
public:
  uint8_t addr = 0x53;
  static constexpr bool     MPU_FACES_NOSE = true;// MPU-6050: componenti verso il muso
  static constexpr float    TAU_S      = 0.5f;    // filtro come il liquido della pallina vera
  static constexpr float    RATE_TAU_S = 0.12f;   // filtro del rateo dal giroscopio
  static constexpr float    FULL_SCALE = 0.25f;   // g a fine corsa
  static constexpr float    WARN_G     = 0.08f;   // pallina fuori dalle righe
  static constexpr uint32_t PERIOD_MS  = 10;      // 100 Hz

  uint8_t lastDevId = 0xFF;                       // ultimo ID letto (diagnosi: 0x68 = MPU-6050, i cloni rispondono anche 0x70, 0x72, 0x98...)
  bool isMpu = false;
  bool begin(TwoWire &w) {
    wire = &w;
    // 1) MPU-6050 (WHO_AM_I 0x75: 0x68 originale; i cloni rispondono anche 0x70, 0x72, 0x98...)
    const uint8_t mc[2] = {0x68, 0x69};
    for (uint8_t c : mc) {
      addr = c;
      if (!present()) continue;
      writeReg(0x6B, 0x80); delay(100);           // reset
      writeReg(0x6B, 0x01);                       // sveglia, clock dal giroscopio X
      delay(10);
      const uint8_t who = readReg(0x75);
      if (who == 0x00 || who == 0xFF) continue;
      lastDevId = who;
      writeReg(0x1A, 0x03);                       // filtro passa basso 44 Hz
      writeReg(0x19, 0x09);                       // 100 Hz
      writeReg(0x1B, 0x08);                       // giroscopio +-500 gradi/s
      writeReg(0x1C, 0x00);                       // accelerometro +-2 g
      isMpu = true;
      break;
    }
    if (!isMpu) return false;                     // solo MPU-6050: nessun altro sensore supportato
    Preferences p;
    if (p.begin("slipball", true)) {
      zeroG = p.getFloat(isMpu ? "zeroM" : "zero", 0.0f);
      if (isMpu && p.isKey("ux")) { up[0] = p.getFloat("ux", 0); up[1] = p.getFloat("uy", 0); up[2] = p.getFloat("uz", 0); upFixed = true; }
      p.end();
    }
    lastMs = millis();
    ok = true;
    return true;
  }
  // Modo remoto: sullo slave l'accelerometro non c'e', e i valori arrivano dal master. La pagina
  // TURN & SLIP non cambia di una riga: legge da qui come se il sensore fosse a bordo.
  void setRemote(bool on, bool fresh, float g, float rate) {
    remoteOn = on; remoteFresh = fresh; remoteG = g; remoteRate = rate;
  }
  bool isRemote() const { return remoteOn; }
  bool remoteStale() const { return remoteOn && !remoteFresh; }
  bool ready() const { return ok || remoteOn; }
  bool hasGyro() const { return (ok && isMpu) || remoteOn; }
  bool gyroReady() const { return remoteOn ? remoteFresh : (hasGyro() && biasOk && upOk()); }
  float turnRate() const { return remoteOn ? remoteRate : rateDps; }   // gradi/s, + = destra
  bool due() const { return ok && millis() - lastMs >= PERIOD_MS; }
  void setOnGround(bool g) { onGround = g; }
  void update() {                                 // chiamare con il bus I2C libero (i2cMutex preso)
    if (!due()) return;
    const uint32_t now = millis(), dtMs = now - lastMs;
    lastMs = now;
    const float dt = dtMs / 1000.0f;
    float raw;
    if (isMpu) {
      float a[3], gy[3];
      if (!readMpu(a, gy)) return;
      for (int i = 0; i < 3; ++i) accF[i] += (accPrimed ? 0.05f : 1.0f) * (a[i] - accF[i]);   // gravita' media
      accPrimed = true;
      calibrate(a, gy);
      if (!upOk()) return;
      // laterale (verso sinistra) = verticale x avanti; avanti = +Z se i componenti guardano il muso
      float lx = up[1], ly = -up[0];
      const float n = sqrtf(lx * lx + ly * ly);
      if (n < 0.3f) return;                       // scheda montata male: Z quasi verticale
      lx /= n; ly /= n;
      if (!MPU_FACES_NOSE) { lx = -lx; ly = -ly; }
      raw = a[0] * lx + a[1] * ly;                // forza laterale verso sinistra = pallina a destra
      if (biasOk) {
        const float w = (gy[0] - bias[0]) * up[0] + (gy[1] - bias[1]) * up[1] + (gy[2] - bias[2]) * up[2];
        const float r = -w;                       // rotazione oraria vista dall'alto = virata a destra
        const float kr = dt / (RATE_TAU_S + dt);
        rateDps += kr * (r - rateDps);
      }
    } else return;                                // senza MPU-6050 non si legge niente
    const float k = dt / (TAU_S + dt);
    if (!primed) { filtG = raw; primed = true; } else filtG += k * (raw - filtG);
  }
  float lateralG() const { return demoOn ? demoG : (remoteOn ? remoteG : filtG - zeroG); }
  void setDemo(bool on, float g = 0.0f) { demoOn = on; demoG = g; }
  float position() const { const float u = lateralG() / FULL_SCALE; return u < -1 ? -1 : (u > 1 ? 1 : u); }
  bool outOfCenter() const { return fabsf(lateralG()) >= WARN_G; }
  // In volo: lo zero del giroscopio si corregge piano piano con il rateo dalla rotta GPS (costante ~30 s).
  // Il giroscopio da' la risposta immediata, il GPS tiene lo zero giusto anche se la temperatura cambia.
  void gpsAid(float gpsRateDps, float dt) {
    if (!gyroReady() || dt <= 0.0f || dt > 0.5f) return;
    if (fabsf(gpsRateDps) > 5.0f || fabsf(rateDps) > 6.0f) return;   // solo in volo dritto o virate leggere
    const float err = constrain(rateDps - gpsRateDps, -3.0f, 3.0f);
    const float k = dt / 30.0f;
    for (int i = 0; i < 3; ++i) bias[i] -= k * err * up[i];             // rate = -(w - bias).up
  }
  void setZero() {                                // in volo dritto e livellato (o a terra in piano)
    Preferences p;
    if (!p.begin("slipball", false)) return;
    if (isMpu && accPrimed) {
      const float n = sqrtf(accF[0] * accF[0] + accF[1] * accF[1] + accF[2] * accF[2]);
      if (n > 0.5f) {
        for (int i = 0; i < 3; ++i) up[i] = accF[i] / n;
        upFixed = true;
        p.putFloat("ux", up[0]); p.putFloat("uy", up[1]); p.putFloat("uz", up[2]);
        primed = false;                           // la pallina riparte dalla nuova verticale
        filtG = 0;
      }
      zeroG = 0;
      p.putFloat("zeroM", zeroG);
    } else {
      zeroG = filtG;
      p.putFloat("zero", zeroG);
    }
    p.end();
  }
  // 0 = coordinata, 1 = SLIP (pallina verso l'interno), 2 = SKID (verso l'esterno)
  int coordination(float turnRateDps) const {
    if (!outOfCenter() || fabsf(turnRateDps) < 0.5f) return 0;
    return ((lateralG() > 0) == (turnRateDps > 0)) ? 1 : 2;
  }
  // Ampolla con il centro in basso; (cx, cy) = centro del fondo.
  void draw(LGFX_Sprite &g, int cx, int cy, int w, int h, int curve) const {
    const uint16_t glass = g.color565(235, 240, 235);
    const uint16_t edge = outOfCenter() ? g.color565(255, 150, 30) : g.color565(140, 140, 140);
    const float ew = outOfCenter() ? 1.6f : 1.1f;
    const int half = w / 2;
    auto yAt = [&](float dx) { const float u = dx / half; return cy - curve * u * u; };
    for (int dx = -half; dx <= half; ++dx) {                 // vetro
      const float yc = yAt(dx);
      g.drawFastVLine(cx + dx, (int)lroundf(yc - h / 2.0f), h, glass);
    }
    for (int k = 0; k < 2; ++k) {                            // bordo sopra e sotto: spezzate sfumate
      const float off = k ? h / 2.0f : -h / 2.0f;
      for (int dx = -half; dx < half; dx += 8) {
        const int dx2 = min(dx + 8, half);
        g.drawWideLine(cx + dx, (int32_t)lroundf(yAt(dx) + off), cx + dx2, (int32_t)lroundf(yAt(dx2) + off), ew, edge);
      }
    }
    for (int sgn = -1; sgn <= 1; sgn += 2)                   // bordi laterali
      g.drawWideLine(cx + sgn * half, (int32_t)lroundf(yAt(half) - h / 2.0f), cx + sgn * half, (int32_t)lroundf(yAt(half) + h / 2.0f), ew, edge);
    const int r = h / 2 - 3;
    for (int sgn = -1; sgn <= 1; sgn += 2) {                 // righe di riferimento
      const int lx = cx + sgn * (r + 3);
      const float yc = yAt(lx - cx);
      g.drawWideLine(lx, (int32_t)lroundf(yc - h / 2.0f + 3), lx, (int32_t)lroundf(yc + h / 2.0f - 3), 1.4f, g.color565(0, 0, 0));
    }
    const float bxf = cx + position() * (half - r - 3);
    const int bx = (int)lroundf(bxf), by = (int)lroundf(yAt(bxf - cx));
    g.fillSmoothCircle(bx, by, r, g.color565(12, 12, 12));   // pallina
    g.fillSmoothCircle(bx - r / 3, by - r / 3, r / 4, g.color565(110, 110, 110));
  }
private:
  TwoWire *wire = nullptr;
  bool ok = false, primed = false, demoOn = false;
  bool remoteOn = false, remoteFresh = false;      // valori dal master (slave senza accelerometro)
  uint32_t lastMs = 0;
  float filtG = 0, zeroG = 0, demoG = 0;
  float remoteG = 0, remoteRate = 0;
  // MPU-6050
  float up[3] = {0, 0, 0};                        // verticale (verso l'alto) nel sistema della scheda
  bool upFixed = false;                           // da SET 0 (salvata) invece che dalla media all'accensione
  float accF[3] = {0, 0, 0};
  bool accPrimed = false;
  float bias[3] = {0, 0, 0};                      // zero del giroscopio
  bool biasOk = false, onGround = true;
  float rateDps = 0;
  float cs[3] = {0, 0, 0}, cs2[3] = {0, 0, 0}, ca[3] = {0, 0, 0};
  int cn = 0;
  bool upOk() const { return up[0] != 0 || up[1] != 0 || up[2] != 0; }
  // Zero del giroscopio: 1 s di strumento fermo (all'accensione, poi a terra ogni tanto). A terra fermo
  // si ricava anche la verticale, se non e' stata fissata con SET 0.
  void calibrate(const float a[3], const float g[3]) {
    if (biasOk && !onGround) { cn = 0; return; }
    for (int i = 0; i < 3; ++i) { cs[i] += g[i]; cs2[i] += g[i] * g[i]; ca[i] += a[i]; }
    if (++cn < 100) return;
    bool still = true;
    float m[3];
    for (int i = 0; i < 3; ++i) {
      m[i] = cs[i] / cn;
      const float var = cs2[i] / cn - m[i] * m[i];
      if (var > 0.6f * 0.6f || (biasOk && fabsf(m[i] - bias[i]) > 3.0f)) still = false;
    }
    if (still) {
      for (int i = 0; i < 3; ++i) bias[i] = biasOk ? bias[i] + 0.3f * (m[i] - bias[i]) : m[i];
      biasOk = true;
      if (!upFixed) {
        const float n = sqrtf(ca[0] * ca[0] + ca[1] * ca[1] + ca[2] * ca[2]);
        if (n > 0.5f * cn) for (int i = 0; i < 3; ++i) up[i] = ca[i] / n;
      }
    }
    cn = 0;
    for (int i = 0; i < 3; ++i) { cs[i] = cs2[i] = ca[i] = 0; }
  }
  bool present() { wire->beginTransmission(addr); return wire->endTransmission(true) == 0; }
  bool readMpu(float acc[3], float gyr[3]) {
    wire->beginTransmission(addr); wire->write(0x3B);
    if (wire->endTransmission(true) != 0) return false;
    if (wire->requestFrom((int)addr, 14) != 14) return false;
    int16_t v[7];
    for (int i = 0; i < 7; ++i) { const uint8_t hi = wire->read(), lo = wire->read(); v[i] = (int16_t)((hi << 8) | lo); }
    for (int i = 0; i < 3; ++i) { acc[i] = v[i] / 16384.0f; gyr[i] = v[4 + i] / 65.5f; }
    return true;
  }
  // Transazioni sempre chiuse con STOP: con il "repeated start" (endTransmission(false)) e il sensore assente
  // la libreria Wire resta bloccata in attesa di requestFrom e gli altri task (BME280, touch) si fermano.
  uint8_t readReg(uint8_t reg) {
    wire->beginTransmission(addr); wire->write(reg);
    if (wire->endTransmission(true) != 0) return 0xFF;
    if (wire->requestFrom((int)addr, 1) != 1) return 0xFF;
    return wire->read();
  }
  void writeReg(uint8_t reg, uint8_t v) { wire->beginTransmission(addr); wire->write(reg); wire->write(v); wire->endTransmission(); }
};
SlipBall slipBall;
bool slipSensorFound() { return slipBall.ready(); }
bool slipPageAvailable() { return slipBall.ready() || demoMode; }

// Lettura della pallina dal loop: solo se il bus I2C e' libero (lo usa anche il SensorTask).
void slipBallService() {
  if (!slipBall.ready()) return;   // non trovato all'accensione: pagina TURN & SLIP nascosta
  if (!slipBall.due() || i2cMutex == nullptr) return;
  if (xSemaphoreTake(i2cMutex, (TickType_t)2) != pdTRUE) return;
  slipBall.setOnGround(!gpsDataValid || gps_ground_speed_kmh < 3.0f);   // zero del giroscopio solo da fermi
  slipBall.update();
  xSemaphoreGive(i2cMutex);
}

// Rateo di virata dalla rotta GPS (gradi/s, + = destra), filtrato. 0 sotto 25 km/h o senza GPS.
static float slipTurnRateDps = 0.0f;
static bool slipTurnRateValid = false;
static void slipUpdateTurnRate() {
  // Campione ogni 100 ms (10 posizioni GPS al secondo); rateo sulla differenza di rotta degli ultimi 0,6 s
  // e filtro leggero: l'aereo segue la virata con circa mezzo secondo di ritardo (prima oltre 1 s).
  const int N = 8;
  static float hc[N]; static unsigned long ht[N];
  static int cnt = 0, head = 0;
  float course = 0, gs = 0; bool valid = false;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)5) == pdTRUE) {
    course = gps_course; gs = gps_ground_speed_kmh; valid = gpsDataValid; xSemaphoreGive(sensorDataMutex);
  }
  const unsigned long now = millis();
  if (!valid || gs < 25.0f) { slipTurnRateValid = false; slipTurnRateDps = 0; cnt = 0; return; }
  if (cnt > 0 && now - ht[(head + N - 1) % N] < 90UL) return;
  hc[head] = course; ht[head] = now; head = (head + 1) % N; if (cnt < N) ++cnt;
  // campione piu' vecchio entro 0,6 s
  int k = -1;
  for (int i = cnt - 1; i >= 1; --i) {
    const int idx = (head + N - 1 - i) % N;
    if (now - ht[idx] <= 650UL) { k = idx; break; }
  }
  if (k < 0) return;
  const float dt = (now - ht[k]) / 1000.0f;
  if (dt < 0.25f) return;
  float d = course - hc[k];
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  const float raw = d / dt;
  if (!slipTurnRateValid) slipTurnRateDps = raw;
  else slipTurnRateDps += 0.4f * (raw - slipTurnRateDps);
  slipTurnRateValid = true;
}

// Dal loop: rateo GPS sempre aggiornato (anche fuori dalla pagina) e correzione dello zero del giroscopio.
void slipGyroAidService() {
  if (!slipBall.hasGyro()) return;
  static unsigned long lastMs = 0;
  const unsigned long now = millis();
  if (now - lastMs < 100UL) return;
  const float dt = lastMs ? (now - lastMs) / 1000.0f : 0.0f;
  lastMs = now;
  slipUpdateTurnRate();
  float gs = 0;
  if (xSemaphoreTake(sensorDataMutex, (TickType_t)2) == pdTRUE) { gs = gps_ground_speed_kmh; xSemaphoreGive(sensorDataMutex); }
  if (slipTurnRateValid && gs >= 40.0f) slipBall.gpsAid(slipTurnRateDps, dt);
}

static unsigned long slipZeroArmMs = 0;            // primo tocco sull'ampolla: SET 0 da confermare
static void slipWideLine(LGFX_Sprite &s, float x0, float y0, float x1, float y1, float r, uint16_t c) { s.drawWideLine(x0, y0, x1, y1, r, c); }
// Aereo visto da dietro (coordinatore di virata), ruotato di ang gradi (+ = ala destra giu').
static void slipDrawPlane(LGFX_Sprite &s, float cx, float cy, float ang, float sc, uint16_t c) {
  // linee con bordi sfumati (anti-aliasing) come nelle altre pagine
  const float a = ang * DEG_TO_RAD, ca = cosf(a), sa = sinf(a);
  auto X = [&](float x, float y) { return (int32_t)lroundf(cx + (x * ca - y * sa) * sc); };
  auto Y = [&](float x, float y) { return (int32_t)lroundf(cy + (x * sa + y * ca) * sc); };
  s.drawWedgeLine(X(0, 0.5f), Y(0, 0.5f), X(167, 2), Y(167, 2), 7.5f * sc, 3.0f * sc, c);     // ala destra
  s.drawWedgeLine(X(0, 0.5f), Y(0, 0.5f), X(-167, 2), Y(-167, 2), 7.5f * sc, 3.0f * sc, c);   // ala sinistra
  s.drawWideLine(X(-63, -26), Y(-63, -26), X(63, -26), Y(63, -26), 3.0f * sc, c);            // piano di coda
  s.drawWedgeLine(X(0, -8), Y(0, -8), X(0, -70), Y(0, -70), 4.0f * sc, 2.0f * sc, c);         // deriva
  s.fillSmoothCircle(lroundf(cx), lroundf(cy), lroundf(24 * sc), c);                           // fusoliera
}

void drawSlipScreen(bool full) {
  (void)full;
  slipUpdateTurnRate();
  // DEMO: virate a destra e a sinistra (fino a 4 gradi/s) con momenti coordinati, in scivolata e in derapata
  float rate = slipTurnRateValid ? slipTurnRateDps : 0.0f;
  bool rateValid = slipTurnRateValid;
  if (slipBall.gyroReady()) { rate = slipBall.turnRate(); rateValid = true; }   // MPU-6050: giroscopio, senza ritardo
  if (demoMode) {
    const float t = millis() / 1000.0f;
    rate = 4.0f * sinf(t * TWO_PI / 16.0f);
    slipBall.setDemo(true, 0.14f * sinf(t * TWO_PI / 7.0f) * fabsf(sinf(t * TWO_PI / 23.0f)));
    rateValid = true;
  } else slipBall.setDemo(false);
  if (!ensureNavigationSprite()) return;
  LGFX_Sprite &s = navigationSprite;
  const uint16_t bg = themeBg();
  const uint16_t white = themeColor(TFT_WHITE), grey = themeColor(TFT_DARKGREY);
  const uint16_t green = themeVivid(TFT_GREEN), orange = themeVivid(TFT_ORANGE), yellow = themeVivid(TFT_YELLOW);
  s.fillScreen(bg);

  // arco del rateo sul bordo del display, mezzo giro in alto: 15 gradi d'arco per grado/s, da -6 a +6
  const int acx = 180, acy = 180, ar = 179;
  auto aOf = [](float v) { return 270.0f + v * 15.0f; };
  const struct { float v0, v1; uint16_t c; } zones[] = {
    {-6, -4, green}, {-4, -2, yellow}, {-2, 2, white}, {2, 4, yellow}, {4, 6, green}};   // colori pieni: si vedono al sole
  // fasce con bordi sfumati (stesso arco delle altre pagine): angoli dal punto delle 3, positivi in alto
  auto aaOf = [](float v) { return 90.0f - v * 15.0f; };
  for (auto &z : zones)
    fillArcAAPart(s, 179.5f, 179.5f, ar - 16, ar, aaOf(z.v1), aaOf(z.v0), false, z.c, bg, z.v1 >= 6.0f, z.v0 <= -6.0f);
  for (int sg = -1; sg <= 1; sg += 2) {   // virata standard: segno verde con bordino nero (si stacca dal giallo)
    fillArcAAPart(s, 179.5f, 179.5f, ar - 25, ar, aaOf(3.0f * sg) - 3.3f, aaOf(3.0f * sg) + 3.3f, false, TFT_BLACK, bg, true, true);
    fillArcAAPart(s, 179.5f, 179.5f, ar - 24, ar, aaOf(3.0f * sg) - 2.5f, aaOf(3.0f * sg) + 2.5f, false, green, TFT_BLACK, true, true);
  }
  const float rc = constrain(rate, -6.0f, 6.0f);
  // tacche di livello e di virata standard (20 gradi), L e R
  const int pcx = 180, pcy = 150;
  for (int sg = -1; sg <= 1; sg += 2) {
    for (int k = 0; k < 2; ++k) {
      const float a = (k ? 20.0f : 0.0f) * DEG_TO_RAD;
      slipWideLine(s, pcx + sg * 112 * cosf(a), pcy + 112 * sinf(a), pcx + sg * 132 * cosf(a), pcy + 132 * sinf(a), 3, white);
    }
  }
  const float a20 = 20.0f * DEG_TO_RAD;
  drawUiVlwSprite(s, "L", pcx - 122 * cosf(a20) - 4, pcy + 122 * sinf(a20) + 24, MC_DATUM, white, bg, 0.6f);
  drawUiVlwSprite(s, "R", pcx + 122 * cosf(a20) + 4, pcy + 122 * sinf(a20) + 24, MC_DATUM, white, bg, 0.6f);
  // Solo l'aereo ha un filtro piu' morbido: attenua le piccole oscillazioni del giroscopio
  // senza ritardare ago, valore numerico o pallina. Tau ~0,45 s.
  static float planeAngle = 0.0f;
  static bool planeAnglePrimed = false;
  static unsigned long planeAngleLastMs = 0;
  const unsigned long planeNow = millis();
  const float planeTarget = constrain(rc * 20.0f / 3.0f, -45.0f, 45.0f);
  if (!planeAnglePrimed) {
    planeAngle = planeTarget;
    planeAnglePrimed = true;
  } else {
    const float planeDt = constrain((planeNow - planeAngleLastMs) / 1000.0f, 0.0f, 0.2f);
    const float planeK = planeDt / (0.45f + planeDt);
    planeAngle += planeK * (planeTarget - planeAngle);
  }
  planeAngleLastMs = planeNow;
  slipDrawPlane(s, pcx, pcy, planeAngle, 0.62f, white);

  // Scritta A.V.E.A* con lo stesso font, colore e riquadro della schermata bussola.
  {
    const int aveaCy = 62, aveaPad = 3;
    static int aveaTop = -1, aveaBot = -1;
    const uint16_t aveaColor = themeColor(TFT_CYAN);
    const uint32_t bgPx = s.readPixelValue(125, aveaCy);
    const int aveaW = drawUiVlwSprite(s, "A.V.E.A*", 180, aveaCy, MC_DATUM, aveaColor, bg, 0.62f);
    if (aveaTop < 0) {
      int top = 10000, bot = -1;
      for (int y = aveaCy - 16; y <= aveaCy + 16; ++y) {
        for (int x = 180 - aveaW / 2 - 2; x <= 180 + aveaW / 2 + 2; ++x) {
          if (s.readPixelValue(x, y) != bgPx) {
            if (y < top) top = y;
            if (y > bot) bot = y;
            break;
          }
        }
      }
      if (bot >= top && bot - top >= 5 && bot - top <= 30) { aveaTop = top; aveaBot = bot; }
    }
    const int boxTop = (aveaTop >= 0 ? aveaTop : aveaCy - 13) - aveaPad - 1;
    const int boxBot = (aveaBot >= 0 ? aveaBot : aveaCy + 13) + aveaPad + 1;
    const int boxW = max(100, aveaW + 16);
    s.drawRoundRect(180 - boxW / 2, boxTop, boxW, boxBot - boxTop + 1, 6, aveaColor);
  }

  // rateo in numeri
  char buf[16];
  if (rateValid) snprintf(buf, sizeof(buf), "%.0f", fabsf(rate)); else snprintf(buf, sizeof(buf), "--");
  drawUiVlwSprite(s, buf, 180, 207, MC_DATUM, white, bg, 1.05f);
  {   // "gradi/s" centrato sotto il valore: nessuna sigla laterale L/R
    const float us = 0.525f;
    const int wU = uiVlwWidthSprite(s, "/s", us);
    const int degW = 10;                               // cerchietto dei gradi (raggio 4) + spazio
    const int total = degW + wU;
    const int x0 = 180 - total / 2;
    s.drawCircle(x0 + 4, 227, 4, grey);                // simbolo dei gradi
    s.drawCircle(x0 + 4, 227, 3, grey);
    drawUiVlwSprite(s, "/s", x0 + degW, 235, ML_DATUM, grey, bg, us);
  }

  // pallina
  if (!slipBall.ready() && !demoMode) {
    drawUiVlwSprite(s, "NO SENSOR", 180, 280, MC_DATUM, themeVivid(TFT_RED), bg, 0.6f);
    char d[32]; snprintf(d, sizeof(d), "MPU-6050  ID 0x%02X", slipBall.lastDevId);
    drawUiVlwSprite(s, d, 180, 310, MC_DATUM, grey, bg, 0.4f);
  } else {
    slipBall.draw(s, 180, 285, 200, 30, 12);   // ampolla 5 px piu' in alto
    const char *msg = "";
    uint16_t mc = green;
    if (slipZeroArmMs != 0 && millis() - slipZeroArmMs < 3000UL) { msg = "TAP AGAIN = SET 0"; mc = themeColor(TFT_YELLOW); }
    // Slave senza dati dal master: la pallina resta dov'e' ed e' giusto dirlo, altrimenti una
    // pallina ferma al centro sembrerebbe un volo coordinato.
    else if (slipBall.remoteStale()) { msg = "NO DATA FROM MASTER"; mc = grey; slipZeroArmMs = 0; }
    else {
      slipZeroArmMs = 0;
      const int co = slipBall.coordination(rate);
      if (co == 1) { msg = slipBall.lateralG() > 0 ? "SLIP > RIGHT RUDDER" : "SLIP > LEFT RUDDER"; mc = orange; }
      else if (co == 2) { msg = slipBall.lateralG() > 0 ? "SKID > RIGHT RUDDER" : "SKID > LEFT RUDDER"; mc = orange; }
      else if (slipBall.outOfCenter()) { msg = slipBall.lateralG() > 0 ? "BALL RIGHT" : "BALL LEFT"; mc = orange; }
      else if (rateValid && fabsf(rate) >= 2.5f && fabsf(rate) <= 3.5f) msg = "STANDARD RATE";
    }
    // Etichette di stato sotto la pallina rimosse su richiesta.
  }
  s.pushSprite(0, 0);
  drawPageLockOverlay();
}
// Tocco sulla pagina: sull'ampolla, due tocchi entro 3 s = SET 0 (in volo dritto e livellato).
void slipHandleTap(int tx, int ty) {
  if (!slipBall.ready() || demoMode || ty < 250 || ty > 315 || tx < 70 || tx > 290) return;
  if (slipZeroArmMs != 0 && millis() - slipZeroArmMs < 3000UL) {
    // Sullo slave la pallina e' quella del master, quindi azzerare qui vuol dire azzerare il
    // master: il comando parte, e il valore nuovo torna col prossimo pacchetto. Come e' montato
    // lo slave non conta niente, perche' l'accelerometro non e' il suo.
    if (slipBall.isRemote()) bleQueueSlipZero();
    else slipBall.setZero();
    slipZeroArmMs = 0;
    playTone(1200, 120);
  } else {
    slipZeroArmMs = millis();
    playTone(1500, 60);
  }
  drawSlipScreen(true);
}
// ===================== fine TURN & SLIP =====================

bool isLockableScreen(int sc) {   // le pagine del giro (non gli avvisi, non le pagine di impostazione nascoste)
  return sc == MAIN_ALT_SCREEN || sc == QFE_SCREEN || sc == VARIOMETER_SCREEN || sc == COMPASS_SCREEN ||
         sc == GO_TO_HOME_SCREEN || sc == GPS_SPEED_SCREEN || sc == GPS_CLOCK_SCREEN || sc == METEO_SCREEN ||
         sc == AIR_DATA_SCREEN || sc == ICT_SCREEN || sc == FLIGHT_TIMER_SCREEN || sc == AUTOMATIC_TIMER_SCREEN ||
         sc == FIRMWARE_SCREEN || sc == GPS_INFO_SCREEN || sc == FUEL_SCREEN || sc == NEAREST_FIELDS_SCREEN ||
         sc == FIELDS_RADAR_SCREEN || sc == SLIP_SCREEN;   // AV SETUP e WIFI LOG no: pagine di impostazione a terra
}
static void drawLockShape(float x, float y, float s, uint16_t c, uint16_t hole) {   // lucchetto chiuso
  lcd.fillArc((int)lroundf(x), (int)lroundf(y - 6 * s), (int)lroundf(5.5f * s), (int)lroundf(8.5f * s), 180.0f, 360.0f, c);
  lcd.fillRect((int)lroundf(x - 8.5f * s), (int)lroundf(y - 6 * s), (int)lroundf(3 * s), (int)lroundf(6 * s), c);
  lcd.fillRect((int)lroundf(x + 5.5f * s), (int)lroundf(y - 6 * s), (int)lroundf(3 * s), (int)lroundf(6 * s), c);
  lcd.fillRoundRect((int)lroundf(x - 11 * s), (int)lroundf(y - 1 * s), (int)lroundf(22 * s), (int)lroundf(17 * s), (int)lroundf(3 * s), c);
  lcd.fillCircle((int)lroundf(x), (int)lroundf(y + 6 * s), (int)lroundf(2.4f * s), hole);
  lcd.fillRect((int)lroundf(x - 1 * s), (int)lroundf(y + 7 * s), max(1, (int)lroundf(2 * s)), (int)lroundf(5 * s), hole);
}
// Lucchetto sopra la pagina: chiamato dopo ogni disegno della pagina e ogni 300 ms dal loop
void drawPageLockOverlay() {
  if (!pageLocked || !isLockableScreen(currentScreen)) return;
  pageLockLastDrawMs = millis();
  const uint16_t red = themeVivid(TFT_RED), bg = themeBg();
  if (pageLockSplashOn) {   // appena bloccata: lucchetto grande al centro
    lcd.fillCircle(180, 180, 70, bg);
    lcd.drawCircle(180, 180, 70, red); lcd.drawCircle(180, 180, 69, red);
    drawLockShape(180, 170, 2.6f, red, bg);
    drawUiVlw("LOCKED", 180, 230, MC_DATUM, red, bg, 0.55f);
    return;
  }
  // in basso a destra, vicino alla cornice: rosso scuro, con un lampeggio rosso acceso di mezzo secondo ogni 3 s
  const bool on = (millis() % 3000UL) < 500UL;
  const uint16_t c = on ? red : lgfx::color565(70, 0, 0);
  lcd.fillCircle(224, 332, 19, bg);
  lcd.drawCircle(224, 332, 19, c); lcd.drawCircle(224, 332, 18, c);
  drawLockShape(224, 328, 0.8f, c, bg);
}
static void togglePageLock() {
  pageLocked = !pageLocked;
  if (pageLocked) {
    pageLockSplashOn = true; pageLockSplashUntil = millis() + 1000UL;
    playTone(1800, 60); delay(70); playTone(2400, 90);   // bip a salire: bloccata
    drawPageLockOverlay();
  } else {
    pageLockSplashOn = false;
    playTone(1400, 60); delay(70); playTone(900, 120);   // bip a scendere: sbloccata
    redrawCurrentScreen();
  }
}
// Nel loop: fine del lucchetto grande e ridisegno periodico del lucchetto piccolo
void pageLockLoop() {
  if (pageLockSplashOn && (long)(millis() - pageLockSplashUntil) >= 0) {
    pageLockSplashOn = false;
    if (pageLocked && isLockableScreen(currentScreen)) redrawCurrentScreen();
  }
  static bool lastOn = false;
  const bool on = (millis() % 3000UL) < 500UL;   // stesso lampeggio di drawPageLockOverlay
  if (pageLocked && (on != lastOn || millis() - pageLockLastDrawMs >= 300UL)) { lastOn = on; drawPageLockOverlay(); }
}

void redrawCurrentScreen() {
  if (!dualModeEnabled) saveScreenPreference();
  switch (currentScreen) {
    case MAIN_ALT_SCREEN:     drawMainScreen(); break;
    case QFE_SCREEN:         drawQfeScreen(); break;
    case VARIOMETER_SCREEN:   drawVariometerScreen(); break;
    case G_METER_SCREEN:      currentScreen = MAIN_ALT_SCREEN; drawMainScreen(); break;   // G-METER rimosso
    case COMPASS_SCREEN:      drawCompassScreen(true); break;
    case GPS_SPEED_SCREEN: drawGpsGroundSpeedScreen(); break;
    case GPS_CLOCK_SCREEN:    drawGpsClockScreen(); break;
    case AV_SETUP_SCREEN:      drawAvSetupScreen(); break;
    case VARIO_FILTER_SCREEN:  drawVarioFilterScreen(); break;
    case DUAL_MODE_SCREEN:     drawDualModeScreen(); break;
    case METEO_SCREEN:        drawMeteoScreen(); break;
    case AIR_DATA_SCREEN:     drawAirDataScreen(); break;
    case ICT_SCREEN:          drawIctScreen(); break;
    case ICT_ALERT_SCREEN:    drawIctAlertScreen(); break;
    case RISK_WARNING_SCREEN: drawRiskWarningScreen(); break;
    case SUNSET_SCREEN:       drawSunsetScreen(lcd, sunsetText); break;
    case FLIGHT_TIMER_SCREEN: drawFlightTimerScreen(); break;
    case AUTOMATIC_TIMER_SCREEN: drawAutomaticFlightTimerScreen(); break;
    case FLIGHT_LOG_SCREEN: drawFlightLogScreen(); break;
    case FLIGHT_END_SCREEN:   drawFlightEndScreen(); break;
    case FIRMWARE_SCREEN:     drawFirmwareScreen(); break;
    case LICENSE_SCREEN:      drawLicenseScreen(); break;
    case AIRSPACE_ALERT_SCREEN: drawAirspaceAlertScreen(); break;
    case DENSITY_ALT_SCREEN:  drawDensityAltScreen(); break;
    case ONLINE_WX_SCREEN:    drawOnlineWxScreen(); break;
    case PAGE_SELECT_SCREEN:  drawPageSelectScreen(); break;
    case WIFI_LOGIN_SCREEN:   drawWifiLoginScreen(); break;
    case AVEA_LOGO_SCREEN:    drawAveaLogoScreen(); break;
    case HOME_FIELD_SCREEN: drawHomeFieldScreen(); break;
    case GO_TO_HOME_SCREEN: drawGoToHomeScreen(); break;
    case NEAREST_FIELDS_SCREEN: updateNearestFieldsScreen(true); break;
    case FIELD_SEARCH_SCREEN: drawFieldSearchScreen(); break;
    case FIELDS_RADAR_SCREEN: updateFieldsRadarScreen(true); break;
    case SLIP_SCREEN:
      if (!slipPageAvailable()) {   // sensore assente: la pagina non si mostra
        currentScreenIndex = nextEnabledScreenIndex(screenIndexOf(SLIP_SCREEN), 1);
        if (screenOrder[currentScreenIndex] == SLIP_SCREEN) currentScreenIndex = screenIndexOf(MAIN_ALT_SCREEN);   // unica pagina attiva
        currentScreen = screenOrder[currentScreenIndex];
        redrawCurrentScreen();
      } else drawSlipScreen(true);
      break;
    case LUBBER_LINE_SCREEN: drawLubberLineScreen(); break;
    case HOME_COORDS_SCREEN: drawHomeCoordsScreen(); break;
    case HOME_COORDS_FMT_SCREEN: drawHomeCoordsFormatScreen(); break;
    case ALT_OFFSET_SCREEN: drawAltOffsetScreen(); break;
    case FUEL_SCREEN: drawFuelScreen(); break;
    case FUEL_RESERVE_SCREEN: drawFuelScreen(); break;
    case FUEL_SETUP_SCREEN: drawFuelSetupScreen(); break;
    case GPS_INFO_SCREEN: drawGpsInfoScreen(); break;
    case WIFI_LOGS_SCREEN: drawWifiLogsScreen(); break;
    case FLIGHT_LOGS_VIEW_SCREEN: {
      int count = getNumSavedLogs();
      if (count > 0) drawFlightLogsViewScreen(currentLogViewIndex);
      else drawNoLogsViewScreen();
    } break;
    default: drawMainScreen(); break;
  }
}

// ========== WIFI: DOWNLOAD DEI LOG VOLO ==========
// Il WiFi resta SPENTO durante il volo: si attiva solo dal tasto WIFI nella
// schermata dei log salvati, si spegne da solo se parte un timer di volo o se
// si lascia la schermata. Lo strumento si collega all'hotspot dello smartphone
// (stazione) e serve una pagina web con i log; l'IP e' mostrato sul display.
// NOTA: una password WPA2 deve avere almeno 8 caratteri (l'hotspot del telefono
// non accetta "avea" da 4): imposta la stessa password sia sul telefono che qui.
// Nome rete e password si cambiano dal dispositivo (WIFI LOG -> CHANGE LOGIN) e restano salvati nel
// namespace NVS "wifi" (non toccato da "Reset preferences"). Questi sono i valori di serie.
static const char* WIFI_LOG_DEFAULT_SSID = "avea";
static const char* WIFI_LOG_DEFAULT_PASS = "aveaavea";
static char wifiLogSsid[33] = "avea";       // nome reale della rete (hotspot del telefono), max 32 caratteri
static char wifiLogPass[64] = "aveaavea";   // password WPA2: 8..63 caratteri (vuota = rete aperta)
static const char* WIFI_LOG_HOST = "vario";
static const unsigned long WIFI_LOG_CONNECT_TIMEOUT_MS = 30000UL;

enum WifiLogState { WIFI_LOG_OFF, WIFI_LOG_CONNECTING, WIFI_LOG_CONNECTED, WIFI_LOG_FAILED };
static WifiLogState wifiLogState = WIFI_LOG_OFF;
static WifiLogState wifiLogDrawnState = WIFI_LOG_OFF;
static unsigned long wifiLogStartedAt = 0;
static ScreenState wifiReturnScreen = ICT_SCREEN;
static bool wifiLogRoutesReady = false;
static WebServer wifiLogServer(80);
static bool wifiOtaUploadActive = false;
static bool wifiOtaUploadFailed = false;
static bool wifiOtaUploadSucceeded = false;
static char wifiOtaErrorMessage[80] = "";

// ========== TRACCIA GPS DEI VOLI (file KML per Google Earth) ==========
// Durante il volo (timer automatico partito) si salva un punto ogni 2 s nella memoria interna (LittleFS,
// partizione "spiffs"): posizione, quota, velocita'. Un file per volo, nominato con data e ora di decollo
// del log; si tengono solo le tracce dei voli ancora presenti nei log (gli ultimi 10). La pagina WIFI LOG
// offre per ogni volo il file .kml: percorso colorato per quota (in 3D) e per velocita', decollo e atterraggio.
// Lo slave non registra: la traccia si scarica dal master.
#define TRACK_DIR "/trk"
static const unsigned long TRACK_SAMPLE_MS = 2000UL;
static const uint32_t TRACK_MAX_POINTS = 21600UL;   // 12 ore a un punto ogni 2 s (circa 340 KB)
static const int TRACK_BUF_POINTS = 32;              // scrittura in memoria ogni 32 punti (circa un minuto)
#pragma pack(push, 1)
struct TrackPoint { int32_t lat6; int32_t lon6; int16_t altM; uint16_t speed10; uint16_t tSec; int16_t vario100; };
#pragma pack(pop)
static_assert(sizeof(TrackPoint) == 16, "TrackPoint deve essere di 16 byte");
static bool trackFsReady = false;
static bool trackActive = false;
static char trackPath[40] = "";
static TrackPoint trackBuf[TRACK_BUF_POINTS];
static int trackBufCount = 0;
static uint32_t trackPoints = 0;
static unsigned long trackLastSampleMs = 0;
static unsigned long trackStartMs = 0;

// Nome del file traccia dalla data ("gg/mm/aaaa") e dall'ora di decollo ("hh:mm:ss") del log.
static bool trackPathFor(const char *date, const char *start, char *out, size_t outSize) {
  char d[9], t[7];
  int k = 0;
  for (const char *p = date; *p && k < 8; ++p) if (isdigit((unsigned char)*p)) d[k++] = *p;
  if (k != 8) return false;
  k = 0;
  for (const char *p = start; *p && k < 6; ++p) if (isdigit((unsigned char)*p)) t[k++] = *p;
  if (k != 6) return false;
  t[6] = '\0';
  snprintf(out, outSize, TRACK_DIR "/%c%c%c%c%c%c%c%c_%s.avt", d[4], d[5], d[6], d[7], d[2], d[3], d[0], d[1], t);
  return true;
}

// Cancella le tracce dei voli non piu' presenti nei log (si tengono solo quelle degli ultimi 10 voli).
static void trackCleanup() {
  if (!trackFsReady || trackActive) return;
  loadFlightLogCache();
  char keep[MAX_LOGS][40];
  int keepCount = 0;
  for (int i = 0; i < flightLogCacheCount && i < MAX_LOGS; ++i) {
    if (trackPathFor(flightLogCache[i].date, flightLogCache[i].startTime, keep[keepCount], sizeof(keep[0]))) ++keepCount;
  }
  String toRemove[24];
  int removeCount = 0;
  File dir = LittleFS.open(TRACK_DIR);
  if (!dir || !dir.isDirectory()) return;
  File f = dir.openNextFile();
  while (f && removeCount < 24) {
    String p = f.path();
    f.close();
    bool kept = false;
    for (int i = 0; i < keepCount; ++i) if (p == keep[i]) { kept = true; break; }
    if (!kept) toRemove[removeCount++] = p;
    f = dir.openNextFile();
  }
  dir.close();
  for (int i = 0; i < removeCount; ++i) {
    LittleFS.remove(toRemove[i]);
    AVEA_LOGF("TRACK: cancellata %s\n", toRemove[i].c_str());
  }
}

void trackMount() {
  trackFsReady = LittleFS.begin(true, "/littlefs", 5, "spiffs");   // la prima volta prepara (formatta) la partizione
  if (!trackFsReady) {
    // Schema di partizioni con "ffat" al posto di "spiffs" (es. 16M Flash 3MB APP/9.9MB FATFS): si usa quella
    trackFsReady = LittleFS.begin(true, "/littlefs", 5, "ffat");
    if (trackFsReady) AVEA_LOGLN("TRACK: memoria interna sulla partizione \"ffat\"");
  }
  if (!trackFsReady) {
    AVEA_LOGLN("TRACK: memoria tracce non disponibile (partizione \"spiffs\" o \"ffat\" assente?)");
    return;
  }
  if (!LittleFS.exists(TRACK_DIR)) LittleFS.mkdir(TRACK_DIR);
  AVEA_LOGF("TRACK: memoria tracce pronta, %u KB usati su %u KB\n",
                (unsigned)(LittleFS.usedBytes() / 1024), (unsigned)(LittleFS.totalBytes() / 1024));
  trackCleanup();
}

static void trackFlush() {
  if (trackBufCount == 0) return;
  File f = LittleFS.open(trackPath, FILE_APPEND);
  if (f) {
    f.write((const uint8_t *)trackBuf, (size_t)trackBufCount * sizeof(TrackPoint));
    f.close();
  } else {
    AVEA_LOGLN("TRACK: scrittura non riuscita");
  }
  trackBufCount = 0;
}

// Al decollo (data e ora di decollo gia' impostate)
void trackStart() {
  trackActive = false;
  if (!trackFsReady || bleRole == BLE_ROLE_SLAVE) return;
  if (!trackPathFor(flightDate, automaticStartUtc, trackPath, sizeof(trackPath))) {
    AVEA_LOGLN("TRACK: data/ora di decollo non disponibili, traccia non registrata");
    return;
  }
  if (LittleFS.exists(trackPath)) LittleFS.remove(trackPath);
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < 400UL * 1024UL) {
    AVEA_LOGLN("TRACK: memoria quasi piena, traccia non registrata");
    return;
  }
  trackBufCount = 0;
  trackPoints = 0;
  trackLastSampleMs = 0;
  trackStartMs = millis();
  trackActive = true;
  AVEA_LOGF("TRACK: registrazione in %s\n", trackPath);
}

// In volo, chiamata spesso: salva un punto ogni TRACK_SAMPLE_MS se il GPS e' valido
void trackSample(unsigned long now, bool gpsValid, double lat, double lon, float speedKmh) {
  if (!trackActive || !gpsValid || trackPoints >= TRACK_MAX_POINTS) return;
  if (trackLastSampleMs != 0 && now - trackLastSampleMs < TRACK_SAMPLE_MS) return;
  if (!isfinite(lat) || !isfinite(lon) || (lat == 0.0 && lon == 0.0)) return;
  float alt = 0.0f, vario = 0.0f;
  if (sensorDataMutex == nullptr || xSemaphoreTake(sensorDataMutex, (TickType_t)5) != pdTRUE) return;
  const bool baroOk = sharedAltitudeValid && isfinite(sharedFilteredAlt);
  alt = baroOk ? sharedFilteredAlt : (float)gps_altitude;   // quota barometrica (QNH), altrimenti quella GPS
  if (sharedVarioValid && isfinite(sharedVarioKalman)) vario = sharedVarioKalman;
  xSemaphoreGive(sensorDataMutex);
  trackLastSampleMs = now;
  TrackPoint &p = trackBuf[trackBufCount++];
  p.lat6 = (int32_t)llround(lat * 1e6);
  p.lon6 = (int32_t)llround(lon * 1e6);
  p.altM = (int16_t)constrain(lroundf(alt), -1000L, 20000L);
  p.speed10 = (uint16_t)constrain(lroundf(speedKmh * 10.0f), 0L, 65535L);
  p.tSec = (uint16_t)min((now - trackStartMs) / 1000UL, 65535UL);
  p.vario100 = (int16_t)constrain(lroundf(vario * 100.0f), -32000L, 32000L);
  ++trackPoints;
  if (trackBufCount >= TRACK_BUF_POINTS) trackFlush();
}

// All'atterraggio (dopo il salvataggio del log)
void trackFinish() {
  if (trackActive) {
    trackFlush();
    trackActive = false;
    AVEA_LOGF("TRACK: volo chiuso, %lu punti\n", (unsigned long)trackPoints);
  }
  trackCleanup();
}

static bool trackExistsForLog(const FlightLog &l) {
  char path[40];
  return trackFsReady && trackPathFor(l.date, l.startTime, path, sizeof(path)) && LittleFS.exists(path);
}

// ---- Scrittura del KML verso il browser, a blocchi ----
static char kmlOut[1400];
static size_t kmlLen = 0;
static void kmlFlush() {
  if (kmlLen == 0) return;
  wifiLogServer.sendContent(kmlOut, kmlLen);
  kmlLen = 0;
  esp_task_wdt_reset();
}
static void kmlPut(const char *s) {
  const size_t n = strlen(s);
  if (kmlLen + n > sizeof(kmlOut)) kmlFlush();
  if (n > sizeof(kmlOut)) { wifiLogServer.sendContent(s, n); return; }
  memcpy(kmlOut + kmlLen, s, n);
  kmlLen += n;
}
static void kmlPrintf(const char *fmt, ...) {
  char b[300];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(b, sizeof(b), fmt, ap);
  va_end(ap);
  kmlPut(b);
}
// 8 colori dal blu (basso / lento) al rosso (alto / veloce)
static const uint8_t KML_RGB[8][3] = {
  {0x30, 0x50, 0xF8}, {0x20, 0xA0, 0xF0}, {0x20, 0xC8, 0xB0}, {0x40, 0xC0, 0x40},
  {0xB0, 0xD0, 0x20}, {0xF0, 0xC0, 0x20}, {0xF0, 0x78, 0x20}, {0xE0, 0x20, 0x20}
};
static int kmlBin(float v, float lo, float hi) {
  if (hi - lo < 1.0f) return 0;
  const int b = (int)((v - lo) / (hi - lo) * 8.0f);
  return b < 0 ? 0 : (b > 7 ? 7 : b);
}
static void kmlCoord(int32_t lat6, int32_t lon6, int16_t altM) {
  kmlPrintf("%.6f,%.6f,%d\n", lon6 / 1e6, lat6 / 1e6, (int)altM);
}

// Una cartella del KML: segmenti colorati per quota (in 3D con la "tenda" fino a terra) o per velocita' (a terra).
static void kmlFolder(File &f, uint32_t n, bool byAlt, float lo, float hi) {
  const char *unit = byAlt ? "m" : "km/h";
  kmlPrintf("<Folder><name>%s</name><visibility>%d</visibility><open>0</open><description><![CDATA[",
            byAlt ? "Altitude (QNH)" : "Ground speed", byAlt ? 1 : 0);
  for (int k = 7; k >= 0; --k) {
    kmlPrintf("<font color=\"#%02X%02X%02X\">&#9632;</font> %.0f - %.0f %s<br>",
              KML_RGB[k][0], KML_RGB[k][1], KML_RGB[k][2],
              lo + (hi - lo) * k / 8.0f, lo + (hi - lo) * (k + 1) / 8.0f, unit);
  }
  kmlPut("]]></description>\n");
  const char *open = byAlt
    ? "<Placemark><visibility>1</visibility><styleUrl>#a%d</styleUrl><LineString><extrude>1</extrude><tessellate>1</tessellate>"
      "<altitudeMode>absolute</altitudeMode><coordinates>\n"
    : "<Placemark><visibility>0</visibility><styleUrl>#s%d</styleUrl><LineString><tessellate>1</tessellate>"
      "<altitudeMode>clampToGround</altitudeMode><coordinates>\n";
  const char *close = "</coordinates></LineString></Placemark>\n";
  f.seek(0);
  TrackPoint chunk[32];
  TrackPoint prev = {};
  bool havePrev = false, isOpen = false;
  int curBin = -1;
  uint32_t done = 0;
  while (done < n) {
    const size_t want = (size_t)min((uint32_t)32, n - done);
    const size_t got = f.read((uint8_t *)chunk, want * sizeof(TrackPoint)) / sizeof(TrackPoint);
    if (got == 0) break;
    for (size_t j = 0; j < got; ++j) {
      const TrackPoint &p = chunk[j];
      const int bin = kmlBin(byAlt ? (float)p.altM : p.speed10 / 10.0f, lo, hi);
      if (!isOpen) {
        kmlPrintf(open, bin);
        if (havePrev) kmlCoord(prev.lat6, prev.lon6, prev.altM);
        isOpen = true;
        curBin = bin;
      } else if (bin != curBin) {
        kmlCoord(p.lat6, p.lon6, p.altM);   // il punto di passaggio sta in entrambi i segmenti: niente buchi
        kmlPut(close);
        kmlPrintf(open, bin);
        curBin = bin;
      }
      kmlCoord(p.lat6, p.lon6, p.altM);
      prev = p;
      havePrev = true;
    }
    done += got;
  }
  if (isOpen) kmlPut(close);
  kmlPut("</Folder>\n");
}

// GET /track.kml?i=N : traccia del log N (0 = volo piu' recente)
static void wifiLogSendKml() {
  const int i = wifiLogServer.hasArg("i") ? wifiLogServer.arg("i").toInt() : -1;
  FlightLog l;
  char path[40];
  if (i < 0 || !readLogFromPrefs(i, l) || !trackFsReady || !trackPathFor(l.date, l.startTime, path, sizeof(path)) ||
      !LittleFS.exists(path)) {
    wifiLogServer.send(404, "text/plain", "Track not available for this flight");
    return;
  }
  File f = LittleFS.open(path, FILE_READ);
  const uint32_t n = f ? (uint32_t)(f.size() / sizeof(TrackPoint)) : 0;
  if (n < 2) {
    if (f) f.close();
    wifiLogServer.send(404, "text/plain", "Track too short");
    return;
  }
  // Primo passaggio: minimi e massimi per la scala dei colori
  float altLo = 1e9f, altHi = -1e9f, spdLo = 1e9f, spdHi = -1e9f;
  TrackPoint first = {}, last = {}, chunk[32];
  uint32_t done = 0;
  while (done < n) {
    const size_t want = (size_t)min((uint32_t)32, n - done);
    const size_t got = f.read((uint8_t *)chunk, want * sizeof(TrackPoint)) / sizeof(TrackPoint);
    if (got == 0) break;
    if (done == 0) first = chunk[0];
    for (size_t j = 0; j < got; ++j) {
      const float a = chunk[j].altM, s = chunk[j].speed10 / 10.0f;
      if (a < altLo) altLo = a;
      if (a > altHi) altHi = a;
      if (s < spdLo) spdLo = s;
      if (s > spdHi) spdHi = s;
    }
    last = chunk[got - 1];
    done += got;
  }
  char fileName[48];
  {
    char key[40];
    trackPathFor(l.date, l.startTime, key, sizeof(key));
    const char *base = strrchr(key, '/');
    snprintf(fileName, sizeof(fileName), "AVEA_%.13s.kml", base ? base + 1 : "track");
  }
  char disp[80];
  snprintf(disp, sizeof(disp), "attachment; filename=\"%s\"", fileName);
  wifiLogServer.sendHeader("Content-Disposition", disp);
  wifiLogServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  wifiLogServer.send(200, "application/vnd.google-earth.kml+xml", "");
  kmlLen = 0;
  kmlPut("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<kml xmlns=\"http://www.opengis.net/kml/2.2\"><Document>\n");
  kmlPrintf("<name>AVEA %s %.5s</name><open>1</open>\n", l.date, l.startTime);
  for (int k = 0; k < 8; ++k) {
    const uint8_t r = KML_RGB[k][0], g = KML_RGB[k][1], b = KML_RGB[k][2];
    kmlPrintf("<Style id=\"a%d\"><LineStyle><color>ff%02x%02x%02x</color><width>4</width></LineStyle>"
              "<PolyStyle><color>55%02x%02x%02x</color></PolyStyle></Style>\n", k, b, g, r, b, g, r);
    kmlPrintf("<Style id=\"s%d\"><LineStyle><color>ff%02x%02x%02x</color><width>5</width></LineStyle></Style>\n",
              k, b, g, r);
  }
  kmlPut("<Style id=\"to\"><IconStyle><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/grn-circle.png</href></Icon></IconStyle></Style>\n");
  kmlPut("<Style id=\"ld\"><IconStyle><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/paddle/red-circle.png</href></Icon></IconStyle></Style>\n");
  // Decollo e atterraggio
  kmlPrintf("<Placemark><name>Takeoff %.5s</name><styleUrl>#to</styleUrl><description>%s</description>"
            "<Point><altitudeMode>absolute</altitudeMode><coordinates>%.6f,%.6f,%d</coordinates></Point></Placemark>\n",
            l.startTime, l.date, first.lon6 / 1e6, first.lat6 / 1e6, (int)first.altM);
  const unsigned long sec = l.durationSec;
  char fuelTxt[16];
  if (l.fuelUsed >= 0.0f && isfinite(l.fuelUsed)) snprintf(fuelTxt, sizeof(fuelTxt), "%.1f l", l.fuelUsed);
  else strcpy(fuelTxt, "-");
  kmlPrintf("<Placemark><name>Landing %.5s</name><styleUrl>#ld</styleUrl><description><![CDATA["
            "Duration %02lu:%02lu:%02lu<br>Distance %.1f km<br>Max altitude %.0f m<br>Max GPS speed %.0f km/h<br>"
            "Max climb %.1f m/s<br>Approx fuel %s]]></description>",
            l.stopTime, sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL, l.distance, l.maxAltitude, l.maxSpeed,
            l.maxClimb, fuelTxt);
  kmlPrintf("<Point><altitudeMode>absolute</altitudeMode><coordinates>%.6f,%.6f,%d</coordinates></Point></Placemark>\n",
            last.lon6 / 1e6, last.lat6 / 1e6, (int)last.altM);
  kmlFolder(f, n, true, altLo, altHi);
  kmlFolder(f, n, false, spdLo, spdHi);
  kmlPut("</Document></kml>\n");
  kmlFlush();
  f.close();
  wifiLogServer.sendContent("");
}

// ========== SPAZI AEREI (file OpenAir, es. openAIP / XCSoar) ==========
// Il file .txt OpenAir si carica dalla pagina web WIFI LOG ("Airspace"): viene convertito in un file
// binario compatto (/asp.bin nella memoria interna) e caricato nella PSRAM all'avvio.
// In volo (timer automatico o manuale attivo), una volta al secondo si controlla la posizione GPS e la
// quota: avviso AIRSPACE quando si e' vicini (distanza orizzontale impostabile) o dentro uno spazio aereo,
// e quando si e' sotto il pavimento entro il margine verticale. Ogni spazio avvisa una volta per
// avvicinamento; un nuovo avviso se si entra. Esclusi: FIR, ASRA (aree attivita' ULM/sportive), NONE.
// Quote: AMSL dal barometro (QNH); FL da quota pressione (1013.25); AGL riferite alla quota del campo
// di decollo (QFE), approssimazione: lo strumento non conosce il terreno sotto di se'.
#define ASP_BIN_PATH "/asp.bin"
#define ASP_TXT_PATH "/asp_upload.txt"
static const uint32_t ASP_MAGIC = 0x31505341UL;   // "ASP1"
static const int ASP_MAX_AREAS = 5000;
enum : uint8_t { ASP_REF_AMSL = 0, ASP_REF_AGL = 1, ASP_REF_FL = 2 };
struct AspArea {
  int32_t minLat, maxLat, minLon, maxLon;   // gradi * 1e6
  uint32_t firstPt;
  uint16_t nPts;
  int32_t lowFt, highFt;
  uint8_t lowRef, highRef;
  char cls[4];
  char type[6];
  char name[34];
};
struct AspPt { int32_t lat6, lon6; };
static AspArea *aspAreas = nullptr;
static AspPt *aspPts = nullptr;
static int aspCount = 0;
static uint32_t aspPtCount = 0;
static uint8_t *aspAlerted = nullptr;          // livello gia' avvisato per ogni spazio (0, 1 vicino, 2 dentro)
bool aspAlertsEnabled = true;
int aspWarnDistM = 2000;                       // distanza orizzontale di avviso
int aspVertMarginFt = 300;                     // margine verticale
static char aspFileInfo[64] = "No airspace file";
static unsigned long aspLastCheckMs = 0;
// Avviso a schermo
static ScreenState aspPreviousScreen = MAIN_ALT_SCREEN;
static unsigned long aspAlertShownMs = 0;
static int aspAlertIdx = -1;
static int aspAlertLevel = 0;
static int aspAlertDistM = 0;
static int aspAlertVert = 0;                   // 0 dentro i limiti, -1 sotto il pavimento, +1 sopra il tetto
static const unsigned long ASP_ALERT_SHOW_MS = 8000UL;

static void aspFreeAll() {
  if (aspAreas) { heap_caps_free(aspAreas); aspAreas = nullptr; }
  if (aspPts) { heap_caps_free(aspPts); aspPts = nullptr; }
  if (aspAlerted) { heap_caps_free(aspAlerted); aspAlerted = nullptr; }
  aspCount = 0;
  aspPtCount = 0;
}
static void *aspAlloc(size_t bytes) {
  void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p;
}

void loadAirspaceSettings() {
  if (preferences.begin("asp", true)) {
    aspAlertsEnabled = preferences.getBool("on", true);
    aspWarnDistM = preferences.getInt("dist", 2000);
    aspVertMarginFt = preferences.getInt("vmar", 300);
    preferences.end();
  }
  aspWarnDistM = constrain(aspWarnDistM, 500, 10000);
  aspVertMarginFt = constrain(aspVertMarginFt, 0, 2000);
}
static void saveAirspaceSettings() {
  if (preferences.begin("asp", false)) {
    preferences.putBool("on", aspAlertsEnabled);
    preferences.putInt("dist", aspWarnDistM);
    preferences.putInt("vmar", aspVertMarginFt);
    preferences.end();
  }
}

// Carica /asp.bin nella PSRAM
bool airspaceLoad() {
  aspFreeAll();
  strcpy(aspFileInfo, "No airspace file");
  if (!trackFsReady || !LittleFS.exists(ASP_BIN_PATH)) return false;
  File f = LittleFS.open(ASP_BIN_PATH, FILE_READ);
  if (!f) return false;
  uint32_t hdr[3] = {0, 0, 0};
  char info[48] = "";
  bool ok = f.read((uint8_t *)hdr, sizeof(hdr)) == sizeof(hdr) && hdr[0] == ASP_MAGIC &&
            hdr[1] > 0 && hdr[1] <= (uint32_t)ASP_MAX_AREAS && hdr[2] > 0 && hdr[2] < 2000000UL &&
            f.read((uint8_t *)info, sizeof(info)) == sizeof(info);
  if (ok) {
    aspAreas = (AspArea *)aspAlloc(hdr[1] * sizeof(AspArea));
    aspPts = (AspPt *)aspAlloc(hdr[2] * sizeof(AspPt));
    aspAlerted = (uint8_t *)aspAlloc(hdr[1]);
    ok = aspAreas && aspPts && aspAlerted &&
         f.read((uint8_t *)aspAreas, hdr[1] * sizeof(AspArea)) == hdr[1] * sizeof(AspArea);
    uint32_t done = 0;
    while (ok && done < hdr[2] * sizeof(AspPt)) {
      const size_t want = min((uint32_t)16384, hdr[2] * (uint32_t)sizeof(AspPt) - done);
      const size_t got = f.read((uint8_t *)aspPts + done, want);
      if (got == 0) { ok = false; break; }
      done += got;
      esp_task_wdt_reset();
    }
  }
  f.close();
  if (!ok) {
    aspFreeAll();
    strcpy(aspFileInfo, "Airspace file not valid");
    AVEA_LOGLN("AIRSPACE: file non valido o PSRAM non disponibile");
    return false;
  }
  aspCount = (int)hdr[1];
  aspPtCount = hdr[2];
  memset(aspAlerted, 0, aspCount);
  info[sizeof(info) - 1] = '\0';
  snprintf(aspFileInfo, sizeof(aspFileInfo), "%d airspaces - %s", aspCount, info);
  AVEA_LOGF("AIRSPACE: %d spazi aerei, %lu punti (%s)\n", aspCount, (unsigned long)aspPtCount, info);
  return true;
}

// ---- Conversione del file OpenAir in binario ----
static bool aspParseCoord(const char *&s, double &out, char posHemi, char negHemi) {
  while (*s == ' ' || *s == '\t' || *s == ',') ++s;
  double parts[3] = {0, 0, 0};
  int np = 0;
  while (np < 3) {
    char *end = nullptr;
    const double v = strtod(s, &end);
    if (end == s) break;
    parts[np++] = v;
    s = end;
    if (*s == ':') ++s; else break;
  }
  if (np == 0) return false;
  while (*s == ' ') ++s;
  const char h = (char)toupper((unsigned char)*s);
  if (h != posHemi && h != negHemi) return false;
  ++s;
  out = parts[0] + parts[1] / 60.0 + parts[2] / 3600.0;
  if (h == negHemi) out = -out;
  return true;
}
static bool aspParseLatLon(const char *&s, double &lat, double &lon) {
  return aspParseCoord(s, lat, 'N', 'S') && aspParseCoord(s, lon, 'E', 'W');
}
// "GND", "SFC", "2000ft AMSL", "FL95", "1000ft AGL", "300m AGL", "UNL"
static void aspParseAlt(const char *s, int32_t &ft, uint8_t &ref) {
  while (*s == ' ') ++s;
  char u[40];
  size_t k = 0;
  for (; s[k] && k < sizeof(u) - 1; ++k) u[k] = (char)toupper((unsigned char)s[k]);
  u[k] = '\0';
  ft = 0;
  ref = ASP_REF_AMSL;
  if (strncmp(u, "GND", 3) == 0 || strncmp(u, "SFC", 3) == 0) { ref = ASP_REF_AGL; return; }
  if (strncmp(u, "UNL", 3) == 0) { ft = 99999; return; }
  if (strncmp(u, "FL", 2) == 0) { ft = (int32_t)(atof(u + 2) * 100.0); ref = ASP_REF_FL; return; }
  char *end = nullptr;
  double v = strtod(u, &end);
  if (end == u) return;
  while (*end == ' ') ++end;
  if (*end == 'M' && end[1] != 'S') v *= 3.28084;   // metri (non "MSL")
  ft = (int32_t)lround(v);
  if (strstr(u, "AGL") || strstr(u, "AGND") || strstr(u, "GND")) ref = ASP_REF_AGL;
}
static bool aspTypeIgnored(const char *type) {
  return strcmp(type, "FIR") == 0 || strcmp(type, "ASRA") == 0 || strcmp(type, "NONE") == 0 ||
         strcmp(type, "UIR") == 0;
}

static AspArea *aspTmpAreas = nullptr;
static AspPt *aspTmpPts = nullptr;
static int aspTmpCount = 0;
static uint32_t aspTmpPtCount = 0, aspTmpPtCap = 0;
static bool aspTmpOpen = false;

static void aspTmpAddPoint(double lat, double lon) {
  if (!aspTmpOpen || aspTmpPtCount >= aspTmpPtCap) return;
  AspArea &a = aspTmpAreas[aspTmpCount];
  if (a.nPts >= 65000) return;
  AspPt &p = aspTmpPts[aspTmpPtCount++];
  p.lat6 = (int32_t)lround(lat * 1e6);
  p.lon6 = (int32_t)lround(lon * 1e6);
  if (a.nPts == 0) { a.minLat = a.maxLat = p.lat6; a.minLon = a.maxLon = p.lon6; }
  if (p.lat6 < a.minLat) a.minLat = p.lat6;
  if (p.lat6 > a.maxLat) a.maxLat = p.lat6;
  if (p.lon6 < a.minLon) a.minLon = p.lon6;
  if (p.lon6 > a.maxLon) a.maxLon = p.lon6;
  ++a.nPts;
}
static void aspTmpClose() {
  if (!aspTmpOpen) return;
  AspArea &a = aspTmpAreas[aspTmpCount];
  aspTmpOpen = false;
  if (a.nPts >= 3 && !aspTypeIgnored(a.type)) ++aspTmpCount;
  else aspTmpPtCount = a.firstPt;   // scartato: libera i suoi punti
}
// Arco da a1 a a2 (gradi, 0 = nord, senso orario se dir > 0) attorno al centro, raggio in NM
static void aspTmpArc(double cLat, double cLon, double rNm, double a1, double a2, int dir) {
  const double rDegLat = rNm * 1852.0 / 111320.0;
  const double cosLat = cos(cLat * DEG_TO_RAD);
  if (cosLat < 0.01) return;
  double sweep = dir > 0 ? a2 - a1 : a1 - a2;
  while (sweep <= 0.0) sweep += 360.0;
  if (sweep > 360.0) sweep = 360.0;
  const int steps = max(2, (int)(sweep / 5.0));
  for (int i = 0; i <= steps; ++i) {
    const double a = (a1 + dir * sweep * i / steps) * DEG_TO_RAD;
    aspTmpAddPoint(cLat + rDegLat * cos(a), cLon + rDegLat * sin(a) / cosLat);
  }
}
static double aspBearing(double cLat, double cLon, double lat, double lon) {
  const double dx = (lon - cLon) * cos(cLat * DEG_TO_RAD), dy = lat - cLat;
  double b = atan2(dx, dy) * RAD_TO_DEG;
  return b < 0 ? b + 360.0 : b;
}
static double aspDistNm(double cLat, double cLon, double lat, double lon) {
  const double dx = (lon - cLon) * cos(cLat * DEG_TO_RAD), dy = lat - cLat;
  return sqrt(dx * dx + dy * dy) * 111320.0 / 1852.0;
}

// Converte il file OpenAir caricato (ASP_TXT_PATH) in ASP_BIN_PATH. Ritorna il numero di spazi o -1.
static int airspaceConvert(const char *sourceName, char *err, size_t errSize) {
  File in = LittleFS.open(ASP_TXT_PATH, FILE_READ);
  if (!in) { snprintf(err, errSize, "Uploaded file not found"); return -1; }
  const size_t fsize = in.size();
  aspTmpPtCap = (uint32_t)min((size_t)600000, fsize / 16 + 1000);
  aspTmpAreas = (AspArea *)aspAlloc(ASP_MAX_AREAS * sizeof(AspArea));
  aspTmpPts = (AspPt *)aspAlloc(aspTmpPtCap * sizeof(AspPt));
  if (!aspTmpAreas || !aspTmpPts) {
    in.close();
    if (aspTmpAreas) heap_caps_free(aspTmpAreas);
    if (aspTmpPts) heap_caps_free(aspTmpPts);
    aspTmpAreas = nullptr; aspTmpPts = nullptr;
    snprintf(err, errSize, "Not enough memory (PSRAM)");
    return -1;
  }
  aspTmpCount = 0; aspTmpPtCount = 0; aspTmpOpen = false;
  double vLat = 0, vLon = 0;
  int vDir = 1;
  char line[200];
  size_t len = 0;
  uint32_t lines = 0;
  bool eof = false;
  while (!eof) {
    const int c = in.read();
    if (c < 0) eof = true;
    if (c >= 0 && c != '\n' && c != '\r') {
      if (len < sizeof(line) - 1) line[len++] = (char)c;
      continue;
    }
    line[len] = '\0';
    len = 0;
    if ((++lines & 1023) == 0) esp_task_wdt_reset();
    const char *s = line;
    while (*s == ' ' || *s == '\t') ++s;
    if (*s == '\0' || *s == '*') continue;
    char *cmt = strchr((char *)s, '*');
    if (cmt) *cmt = '\0';
    const char c0 = (char)toupper((unsigned char)s[0]), c1 = (char)toupper((unsigned char)s[1]);
    const char *arg = s + 2;
    while (*arg == ' ') ++arg;
    if (c0 == 'A' && c1 == 'C') {
      aspTmpClose();
      if (aspTmpCount >= ASP_MAX_AREAS) break;
      AspArea &a = aspTmpAreas[aspTmpCount];
      memset(&a, 0, sizeof(a));
      a.firstPt = aspTmpPtCount;
      a.highFt = 99999;
      a.lowRef = ASP_REF_AGL;
      strncpy(a.cls, arg, sizeof(a.cls) - 1);
      strncpy(a.type, arg, sizeof(a.type) - 1);   // senza AY il tipo e' la classe (formato OpenAir 1)
      aspTmpOpen = true;
      vDir = 1;
    } else if (!aspTmpOpen) {
      continue;
    } else if (c0 == 'A' && c1 == 'Y') {
      AspArea &a = aspTmpAreas[aspTmpCount];
      memset(a.type, 0, sizeof(a.type));
      strncpy(a.type, arg, sizeof(a.type) - 1);
      for (char *p = a.type; *p; ++p) *p = (char)toupper((unsigned char)*p);
    } else if (c0 == 'A' && c1 == 'N') {
      AspArea &a = aspTmpAreas[aspTmpCount];
      strncpy(a.name, arg, sizeof(a.name) - 1);
    } else if (c0 == 'A' && c1 == 'L') {
      AspArea &a = aspTmpAreas[aspTmpCount];
      aspParseAlt(arg, a.lowFt, a.lowRef);
    } else if (c0 == 'A' && c1 == 'H') {
      AspArea &a = aspTmpAreas[aspTmpCount];
      aspParseAlt(arg, a.highFt, a.highRef);
    } else if (c0 == 'D' && c1 == 'P') {
      double lat, lon;
      const char *p = arg;
      if (aspParseLatLon(p, lat, lon)) aspTmpAddPoint(lat, lon);
    } else if (c0 == 'V' && (c1 == ' ' || c1 == '\t')) {
      const char *p = arg;
      while (*p == ' ') ++p;
      const char k = (char)toupper((unsigned char)*p);
      const char *eq = strchr(p, '=');
      if (eq) {
        const char *q = eq + 1;
        if (k == 'X') aspParseLatLon(q, vLat, vLon);
        else if (k == 'D') { while (*q == ' ') ++q; vDir = (*q == '-') ? -1 : 1; }
      }
    } else if (c0 == 'D' && c1 == 'C') {
      const double r = atof(arg);
      if (r > 0.0) aspTmpArc(vLat, vLon, r, 0.0, 360.0, 1);
    } else if (c0 == 'D' && c1 == 'A') {
      double r = 0, a1 = 0, a2 = 0;
      if (sscanf(arg, "%lf , %lf , %lf", &r, &a1, &a2) == 3 && r > 0.0) aspTmpArc(vLat, vLon, r, a1, a2, vDir);
    } else if (c0 == 'D' && c1 == 'B') {
      double lat1, lon1, lat2, lon2;
      const char *p = arg;
      if (aspParseLatLon(p, lat1, lon1)) {
        while (*p == ' ' || *p == ',') ++p;
        if (aspParseLatLon(p, lat2, lon2)) {
          const double r = aspDistNm(vLat, vLon, lat1, lon1);
          aspTmpArc(vLat, vLon, r, aspBearing(vLat, vLon, lat1, lon1), aspBearing(vLat, vLon, lat2, lon2), vDir);
        }
      }
    }
  }
  aspTmpClose();
  in.close();
  int result = aspTmpCount;
  if (aspTmpCount == 0) {
    snprintf(err, errSize, "No airspaces found: is it an OpenAir .txt file?");
    result = -1;
  } else {
    File out = LittleFS.open(ASP_BIN_PATH, FILE_WRITE);
    char info[48];
    memset(info, 0, sizeof(info));
    int hh = 0, mi = 0, ss = 0, dd = 0, mm = 0, yy = 0;
    if (getLocalClock(hh, mi, ss, dd, mm, yy)) snprintf(info, sizeof(info), "%.30s %02d/%02d/%04d", sourceName, dd, mm, yy);
    else snprintf(info, sizeof(info), "%.40s", sourceName);
    const uint32_t hdr[3] = { ASP_MAGIC, (uint32_t)aspTmpCount, aspTmpPtCount };
    bool ok = out && out.write((const uint8_t *)hdr, sizeof(hdr)) == sizeof(hdr) &&
              out.write((const uint8_t *)info, sizeof(info)) == sizeof(info) &&
              out.write((const uint8_t *)aspTmpAreas, aspTmpCount * sizeof(AspArea)) == aspTmpCount * sizeof(AspArea);
    uint32_t done = 0;
    const uint32_t total = aspTmpPtCount * (uint32_t)sizeof(AspPt);
    while (ok && done < total) {
      const size_t n = min((uint32_t)8192, total - done);
      ok = out.write((const uint8_t *)aspTmpPts + done, n) == n;
      done += n;
      esp_task_wdt_reset();
    }
    if (out) out.close();
    if (!ok) { snprintf(err, errSize, "Write error (memory full?)"); LittleFS.remove(ASP_BIN_PATH); result = -1; }
    AVEA_LOGF("AIRSPACE: convertiti %d spazi aerei, %lu punti\n", aspTmpCount, (unsigned long)aspTmpPtCount);
  }
  heap_caps_free(aspTmpAreas);
  heap_caps_free(aspTmpPts);
  aspTmpAreas = nullptr; aspTmpPts = nullptr;
  LittleFS.remove(ASP_TXT_PATH);
  return result;
}

// ---- Controllo in volo ----
static int32_t aspLimitFt(int32_t ft, uint8_t ref, float groundFt, float qnhHpa) {
  if (ref == ASP_REF_AGL) return (int32_t)(ft + groundFt);
  if (ref == ASP_REF_FL) return (int32_t)(ft - (1013.25f - qnhHpa) * 27.5f);   // FL -> quota QNH
  return ft;
}
// Distanza in metri dal bordo e "dentro / fuori" (proiezione piana locale)
// (indice e non AspArea&: i prototipi generati da Arduino stanno prima della definizione della struttura)
static bool aspPolyTest(int idx, double lat, double lon, float &edgeDistM) {
  const AspArea &a = aspAreas[idx];
  const double kx = 111320.0 * cos(lat * DEG_TO_RAD), ky = 111320.0;
  bool inside = false;
  double best = 1e12;
  const AspPt *p = aspPts + a.firstPt;
  for (uint16_t i = 0, j = a.nPts - 1; i < a.nPts; j = i++) {
    const double xi = (p[i].lon6 / 1e6 - lon) * kx, yi = (p[i].lat6 / 1e6 - lat) * ky;
    const double xj = (p[j].lon6 / 1e6 - lon) * kx, yj = (p[j].lat6 / 1e6 - lat) * ky;
    if ((yi > 0) != (yj > 0) && 0 < (xj - xi) * (0 - yi) / (yj - yi) + xi) inside = !inside;
    const double dx = xj - xi, dy = yj - yi, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? -(xi * dx + yi * dy) / l2 : 0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    const double ex = xi + t * dx, ey = yi + t * dy, d2 = ex * ex + ey * ey;
    if (d2 < best) best = d2;
  }
  edgeDistM = (float)sqrt(best);
  return inside;
}

// Pagina FIELDS: CTR attorno all'aereo, rotta in alto, nei colori della carta VFR:
// riempimento rosa chiaro, banda interna rosa piu' scura (6 px), bordo blu tratteggiato. Nessun file: niente.
static uint8_t aspCtrMask[360 * 45];   // 1 bit per pixel del display: dentro / fuori il CTR in disegno
static inline bool aspMaskIn(int x, int y) {
  if (x < 0 || x > 359 || y < 0 || y > 359) return true;   // fuori dal display: nessuna banda sul bordo
  return (aspCtrMask[y * 45 + (x >> 3)] >> (x & 7)) & 1;
}
void aspDrawCtrOutlines(lgfx::LGFXBase &g, double lat, double lon, float course, float rangeKm, float pxPerKm) {
  if (aspCount == 0 || aspAreas == nullptr) return;
  static int16_t sx[1024], sy[1024];
  const double kLon = cos(lat * DEG_TO_RAD);
  const float cs = cosf(course * DEG_TO_RAD), sn = sinf(course * DEG_TO_RAD);
  const int32_t lat6 = (int32_t)lround(lat * 1e6), lon6 = (int32_t)lround(lon * 1e6);
  const int32_t padLat = (int32_t)(rangeKm * 1300.0f / 0.1112f), padLon = (int32_t)(rangeKm * 1300.0f / (0.1112f * max(kLon, 0.1)));   // km -> milionesimi di grado
  const uint16_t cFill = lgfx::color565(243, 205, 208), cBand = lgfx::color565(229, 154, 162), cDash = lgfx::color565(59, 91, 154);
  for (int i = 0; i < aspCount; ++i) {
    const AspArea &a = aspAreas[i];
    if (strcmp(a.type, "CTR") != 0 || a.nPts < 3 || a.nPts > 1024) continue;
    if (lat6 < a.minLat - padLat || lat6 > a.maxLat + padLat || lon6 < a.minLon - padLon || lon6 > a.maxLon + padLon) continue;
    const AspPt *p = aspPts + a.firstPt;
    const int np = a.nPts;
    for (int j = 0; j < np; ++j) {
      const float dx = (float)((p[j].lon6 / 1e6 - lon) * kLon * 111.2), dy = (float)((p[j].lat6 / 1e6 - lat) * 111.2);
      sx[j] = (int16_t)constrain(lroundf(180.0f + (dx * cs - dy * sn) * pxPerKm), -8000L, 8000L);
      sy[j] = (int16_t)constrain(lroundf(180.0f - (dx * sn + dy * cs) * pxPerKm), -8000L, 8000L);
    }
    // maschera dell'interno (riempimento a righe orizzontali)
    memset(aspCtrMask, 0, sizeof(aspCtrMask));
    bool any = false;
    for (int y = 0; y < 360; ++y) {
      int xs[32], nx = 0;
      const float yc = y + 0.5f;
      for (int j = 0, k = np - 1; j < np; k = j++) {
        if ((sy[j] <= yc) != (sy[k] <= yc) && nx < 32)
          xs[nx++] = (int)lroundf(sx[k] + (yc - sy[k]) * (float)(sx[j] - sx[k]) / (float)(sy[j] - sy[k]));
      }
      for (int u = 1; u < nx; ++u) { const int v = xs[u]; int w = u - 1; while (w >= 0 && xs[w] > v) { xs[w + 1] = xs[w]; --w; } xs[w + 1] = v; }
      for (int u = 0; u + 1 < nx; u += 2) {
        const int x0 = max(0, xs[u]), x1 = min(359, xs[u + 1] - 1);
        for (int x = x0; x <= x1; ++x) aspCtrMask[y * 45 + (x >> 3)] |= (uint8_t)(1 << (x & 7));
        if (x1 >= x0) any = true;
      }
    }
    if (!any) {
      // CTR visibile solo come bordo (es. vertice fuori schermo): niente riempimento
    } else {
      // riempimento e banda interna, a tratti orizzontali dello stesso colore
      for (int y = 0; y < 360; ++y) {
        int runX = -1; uint16_t runC = 0;
        for (int x = 0; x <= 360; ++x) {
          uint16_t c = 0;
          const bool in = x < 360 && ((aspCtrMask[y * 45 + (x >> 3)] >> (x & 7)) & 1);
          if (in) c = (!aspMaskIn(x - 6, y) || !aspMaskIn(x + 6, y) || !aspMaskIn(x, y - 6) || !aspMaskIn(x, y + 6)) ? cBand : cFill;
          if (runX >= 0 && (!in || c != runC)) { g.drawFastHLine(runX, y, x - runX, runC); runX = -1; }
          if (in && runX < 0) { runX = x; runC = c; }
        }
      }
    }
    // bordo blu tratteggiato (10 px pieno, 6 px vuoto)
    float phase = 0.0f;
    for (int j = 0; j < np; ++j) {
      const int k = (j + 1) % np;
      const float x0 = sx[j], y0 = sy[j], x1 = sx[k], y1 = sy[k];
      const float len = sqrtf((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
      if (len < 0.5f || len > 4000.0f) continue;
      float t = 0.0f;
      while (t < len) {
        const float cyc = fmodf(phase + t, 16.0f);
        const float seg = cyc < 10.0f ? min(10.0f - cyc, len - t) : min(16.0f - cyc, len - t);
        if (cyc < 10.0f) {
          const float ax = x0 + (x1 - x0) * t / len, ay = y0 + (y1 - y0) * t / len;
          const float bx = x0 + (x1 - x0) * (t + seg) / len, by = y0 + (y1 - y0) * (t + seg) / len;
          g.drawWideLine(ax, ay, bx, by, 1.3f, cDash);
        }
        t += seg;
      }
      phase = fmodf(phase + len, 16.0f);
    }
  }
}

static void aspFormatLimit(char *out, size_t n, int32_t ft, uint8_t ref) {
  if (ref == ASP_REF_AGL && ft == 0) snprintf(out, n, "GND");
  else if (ft >= 99999) snprintf(out, n, "UNL");
  else if (ref == ASP_REF_FL) snprintf(out, n, "FL%ld", (long)(ft / 100));
  else snprintf(out, n, "%ld ft %s", (long)ft, ref == ASP_REF_AGL ? "AGL" : "AMSL");
}

void drawAirspaceAlertScreen() {
  if (aspAlertIdx < 0 || aspAlertIdx >= aspCount) return;
  const AspArea &a = aspAreas[aspAlertIdx];
  const bool inside = aspAlertLevel >= 2;
  const uint16_t bg = inside ? TFT_RED : 0xFC60;   // rosso dentro, arancione vicino
  const uint16_t fg = inside ? TFT_WHITE : TFT_BLACK;
  lcd.fillScreen(bg);
  drawUiVlw("AIRSPACE", 180, 58, MC_DATUM, fg, bg, 1.0f);
  char line[48];
  if (strcmp(a.type, a.cls) == 0 || a.cls[0] == '\0' || strcmp(a.cls, "UNC") == 0) snprintf(line, sizeof(line), "%s", a.type);
  else snprintf(line, sizeof(line), "%s  class %s", a.type, a.cls);
  drawUiVlw(line, 180, 104, MC_DATUM, fg, bg, 0.8f);
  // Nome su due righe al massimo
  char n1[20] = "", n2[20] = "";
  const size_t nl = strlen(a.name);
  if (nl <= 18) {
    strcpy(n1, a.name);
  } else {
    int cut = 18;
    for (int i = 18; i > 6; --i) if (a.name[i] == ' ') { cut = i; break; }
    strncpy(n1, a.name, cut); n1[cut] = '\0';
    const char *r = a.name + cut;
    while (*r == ' ') ++r;
    strncpy(n2, r, 18); n2[18] = '\0';
  }
  drawUiVlw(n1, 180, 146, MC_DATUM, fg, bg, 0.7f);
  if (n2[0]) drawUiVlw(n2, 180, 174, MC_DATUM, fg, bg, 0.7f);
  char lo[20], hi[20];
  aspFormatLimit(lo, sizeof(lo), a.lowFt, a.lowRef);
  aspFormatLimit(hi, sizeof(hi), a.highFt, a.highRef);
  snprintf(line, sizeof(line), "%s - %s", lo, hi);
  drawUiVlw(line, 180, 214, MC_DATUM, fg, bg, 0.6f);
  if (inside) {
    drawUiVlw("INSIDE", 180, 266, MC_DATUM, fg, bg, 1.0f);
  } else if (aspAlertVert < 0 && aspAlertDistM == 0) {
    drawUiVlw("BELOW FLOOR", 180, 266, MC_DATUM, fg, bg, 0.9f);
  } else if (aspAlertVert > 0 && aspAlertDistM == 0) {
    drawUiVlw("ABOVE TOP", 180, 266, MC_DATUM, fg, bg, 0.9f);
  } else {
    snprintf(line, sizeof(line), "%.1f km", aspAlertDistM / 1000.0f);
    drawUiVlw(line, 180, 266, MC_DATUM, fg, bg, 1.0f);
  }
}

void dismissAirspaceAlert() {
  if (currentScreen != AIRSPACE_ALERT_SCREEN) return;
  currentScreen = aspPreviousScreen;
  lastTouchMillis = millis();
  redrawCurrentScreen();
}

// Chiamata nel loop: controllo una volta al secondo e chiusura automatica dell'avviso
// CTR: tipo "CTR" (AY / AC CTR) oppure nome con "CTR" (file OpenAir dove AC e' solo la classe)
static bool aspIsCtr(const char *type, const char *name) {
  if (strcmp(type, "CTR") == 0) return true;
  for (const char *p = name; p[0] && p[1] && p[2]; ++p)
    if (toupper((unsigned char)p[0]) == 'C' && toupper((unsigned char)p[1]) == 'T' && toupper((unsigned char)p[2]) == 'R') return true;
  return false;
}
void checkAirspace() {
  const unsigned long now = millis();
  if (currentScreen == AIRSPACE_ALERT_SCREEN && now - aspAlertShownMs >= ASP_ALERT_SHOW_MS) {
    dismissAirspaceAlert();
    return;
  }
  if (aspCount == 0 || !aspAlertsEnabled) return;
  if (now - aspLastCheckMs < 1000UL) return;
  aspLastCheckMs = now;
  const bool inFlight = flightTimerRunning || automaticFlightRunning;
  if (!inFlight) { memset(aspAlerted, 0, aspCount); return; }   // a terra nessun avviso
  double lat = 0, lon = 0;
  float altM = NAN;
  bool posOk = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    posOk = gpsDataValid;
    lat = gps_latitude;
    lon = gps_longitude;
    if (sharedAltitudeValid && isfinite(sharedFilteredAlt)) altM = sharedFilteredAlt;
    else if (gpsDataValid) altM = gps_altitude;
    xSemaphoreGive(sensorDataMutex);
  }
  if (!posOk || !isfinite(altM) || (lat == 0.0 && lon == 0.0)) return;
  const float altFt = altM * 3.28084f;
  const float groundFt = qfeReferenceValid ? qfeReferenceAltitudeMeters * 3.28084f : 0.0f;
  const float qnh = seaLevelPressure;
  const int32_t lat6 = (int32_t)lround(lat * 1e6), lon6 = (int32_t)lround(lon * 1e6);
  const double cosLat = cos(lat * DEG_TO_RAD);
  const int32_t clearM = aspWarnDistM * 3 / 2;
  const int32_t padLat = (int32_t)(clearM / 0.11132), padLon = (int32_t)(clearM / (0.11132 * max(cosLat, 0.1)));
  int bestIdx = -1, bestLevel = 0, bestDist = 0, bestVert = 0;
  for (int i = 0; i < aspCount; ++i) {
    const AspArea &a = aspAreas[i];
    if (aspOnlyCtr && !aspIsCtr(a.type, a.name)) { aspAlerted[i] = 0; continue; }   // ONLY CTR: le altre zone senza avviso
    if (lat6 < a.minLat - padLat || lat6 > a.maxLat + padLat || lon6 < a.minLon - padLon || lon6 > a.maxLon + padLon) {
      aspAlerted[i] = 0;
      continue;
    }
    const float lowFt = aspLimitFt(a.lowFt, a.lowRef, groundFt, qnh);
    const float highFt = a.highFt >= 99999 ? 1e9f : aspLimitFt(a.highFt, a.highRef, groundFt, qnh);
    float edge = 0;
    const bool in = aspPolyTest(i, lat, lon, edge);
    const bool vIn = altFt >= lowFt && altFt <= highFt;
    const bool vNear = altFt >= lowFt - aspVertMarginFt && altFt <= highFt + aspVertMarginFt;
    const int vert = altFt < lowFt ? -1 : (altFt > highFt ? 1 : 0);
    int level = 0;
    if (in && vIn) level = 2;
    else if ((in && vNear) || (!in && edge <= aspWarnDistM && vNear)) level = 1;
    // Reset con isteresi: lontano o fuori quota con margine piu' ampio
    const bool vClear = altFt < lowFt - aspVertMarginFt * 1.5f || altFt > highFt + aspVertMarginFt * 1.5f;
    if (level == 0 && ((!in && edge > clearM) || vClear)) aspAlerted[i] = 0;
    if (level > aspAlerted[i] &&
        (level > bestLevel || (level == bestLevel && (in ? 0 : (int)edge) < bestDist))) {
      bestIdx = i; bestLevel = level; bestDist = in ? 0 : (int)edge; bestVert = vert;
    }
  }
  if (bestIdx < 0) return;
  // Non sopra altri avvisi a tutto schermo o pagine di inserimento
  if (startupSequenceActive || currentScreen == RISK_WARNING_SCREEN || currentScreen == ICT_ALERT_SCREEN ||
      currentScreen == FLIGHT_END_SCREEN || currentScreen == SUNSET_SCREEN || currentScreen == FUEL_RESERVE_SCREEN ||
      currentScreen == LICENSE_SCREEN || currentScreen == WIFI_LOGS_SCREEN || currentScreen == WIFI_LOGIN_SCREEN ||
      currentScreen == DENSITY_ALT_SCREEN) return;
  if (alertsDeferred()) return;   // vicino alla meta: rimandato (non segnato come gia' avvisato)
  aspAlerted[bestIdx] = (uint8_t)bestLevel;
  if (currentScreen != AIRSPACE_ALERT_SCREEN) aspPreviousScreen = currentScreen;
  aspAlertIdx = bestIdx;
  aspAlertLevel = bestLevel;
  aspAlertDistM = bestDist;
  aspAlertVert = bestVert;
  aspAlertShownMs = now;
  if (isTouching) { ignoreTouchUntilRelease = true; isTouching = false; }
  currentScreen = AIRSPACE_ALERT_SCREEN;
  drawAirspaceAlertScreen();
  AVEA_LOGF("AIRSPACE: %s %s (%s) dist %d m\n", bestLevel >= 2 ? "DENTRO" : "vicino",
                aspAreas[bestIdx].name, aspAreas[bestIdx].type, bestDist);
  if (bestLevel >= 2) { playTone(900, 200); playTone(1500, 200); playTone(900, 200); playTone(1500, 200); }
  else { playTone(1200, 120); playTone(1600, 120); }
}

// ---- Pagina web: caricamento del file e impostazioni ----
static bool aspUploadOk = false;
static char aspUploadErr[64] = "";
static char aspUploadName[40] = "";
static File aspUploadFile;

// Link della pagina web Airspace (si aprono sul telefono, che ha internet tramite il suo hotspot)
#define ASP_LINK_DRIVE "https://drive.google.com/drive/u/0/folders/1vRXCJjZf-rTIVcHebrR4INtRvPpX6Bbm"
#define ASP_LINK_OPENAIP "https://www.openaip.net/data/exports?page=1&amp;limit=200&amp;sortBy=country&amp;sortDesc=false&amp;contentType=airspace"
static void wifiAirspacePage() {
  if (wifiLogServer.hasArg("set")) {
    aspAlertsEnabled = wifiLogServer.hasArg("on");
    if (wifiLogServer.hasArg("dist")) aspWarnDistM = constrain((int)(wifiLogServer.arg("dist").toFloat() * 1000.0f), 500, 10000);
    if (wifiLogServer.hasArg("vmar")) aspVertMarginFt = constrain(wifiLogServer.arg("vmar").toInt(), 0, 2000);
    saveAirspaceSettings();
  }
  if (wifiLogServer.hasArg("del")) {
    LittleFS.remove(ASP_BIN_PATH);
    airspaceLoad();
  }
  static char buf[2800];   // static: fuori dallo stack del server web
  snprintf(buf, sizeof(buf),
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Airspace</title><style>body{font:17px sans-serif;max-width:540px;margin:20px auto;padding:0 14px;background:#111;color:#eee}.bl{color:#fd4;font-weight:bold;font-size:15px;animation:bl 1s steps(2,start) infinite}@keyframes bl{to{visibility:hidden}}"
    ".c{background:#222;border-radius:10px;padding:12px;margin:12px 0}input,button,select{font:17px sans-serif;margin:8px 0}"
    "button{padding:10px 16px;background:#1685ff;color:#fff;border:0;border-radius:8px}a{color:#6cf}"
    "a.l{display:block;margin:8px 0;padding:10px 12px;background:#333;border-radius:8px;text-decoration:none}</style></head><body>"
    "<h2>Airspace</h2><div class=c><b>Loaded:</b> %s</div>"
    "<div class=c><form method=POST action=/airspace enctype=multipart/form-data>"
    "<p>OpenAir file (.txt), e.g. openAIP / XCSoar. Upload on the ground.</p>"
    "<input type=file name=asp accept='.txt,.air,text/plain' required><br>"
    "<a class=l target=_blank rel=noopener href='" ASP_LINK_DRIVE "'>&#128193; Shared folder (Google Drive)</a>"
    "<a class=l target=_blank rel=noopener href='" ASP_LINK_OPENAIP "'>&#127760; openAIP exports (OpenAIR Version 2 .txt)</a>"
    "<p class=bl>Download the file to the phone from one of the links, then choose it above.</p>"
    "<button type=submit>Upload airspace</button></form></div>"
    "<div class=c><form method=GET action=/airspace><input type=hidden name=set value=1>"
    "<label><input type=checkbox name=on %s> Airspace alerts in flight</label><br>"
    "Horizontal warning distance (km) <input name=dist type=number step=0.5 min=0.5 max=10 value=%.1f style='width:80px'><br>"
    "Vertical margin (ft) <input name=vmar type=number step=50 min=0 max=2000 value=%d style='width:80px'><br>"
    "<button type=submit>Save settings</button></form></div>"
    "<p><a href=/airspace?del=1 onclick=\"return confirm('Delete airspace file?')\">Delete airspace file</a></p>"
    "<p><a href=/>&larr; Flight logs</a></p>"
    "<p style='color:#aaa;font-size:14px'>Alerts are an aid only: always check official charts and NOTAMs. "
    "AGL limits use the take-off field altitude.</p></body></html>",
    aspFileInfo, aspAlertsEnabled ? "checked" : "", aspWarnDistM / 1000.0f, aspVertMarginFt);
  wifiLogServer.send(200, "text/html; charset=utf-8", buf);
}

static void wifiAirspaceUpload() {
  HTTPUpload &up = wifiLogServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    aspUploadOk = false;
    aspUploadErr[0] = '\0';
    strncpy(aspUploadName, up.filename.c_str(), sizeof(aspUploadName) - 1);
    aspUploadName[sizeof(aspUploadName) - 1] = '\0';
    if (flightTimerRunning || automaticFlightRunning) { strcpy(aspUploadErr, "Not available during flight"); return; }
    if (!trackFsReady) { strcpy(aspUploadErr, "Internal memory not available"); return; }
    aspUploadFile = LittleFS.open(ASP_TXT_PATH, FILE_WRITE);
    if (!aspUploadFile) strcpy(aspUploadErr, "Cannot write to internal memory");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (aspUploadFile && aspUploadErr[0] == '\0') {
      if (aspUploadFile.write(up.buf, up.currentSize) != up.currentSize) strcpy(aspUploadErr, "Memory full");
      esp_task_wdt_reset();
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (aspUploadFile) aspUploadFile.close();
    aspUploadOk = aspUploadErr[0] == '\0';
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (aspUploadFile) aspUploadFile.close();
    strcpy(aspUploadErr, "Upload interrupted");
  }
}

static void wifiAirspaceFinish() {
  char msg[160];
  if (!aspUploadOk) {
    snprintf(msg, sizeof(msg), "Upload failed: %s", aspUploadErr[0] ? aspUploadErr : "unknown error");
  } else {
    char err[64] = "";
    aspFreeAll();   // libera la PSRAM per la conversione
    const int n = airspaceConvert(aspUploadName, err, sizeof(err));
    airspaceLoad();
    if (n > 0) snprintf(msg, sizeof(msg), "Loaded %d airspaces", n);
    else snprintf(msg, sizeof(msg), "Conversion failed: %s", err);
  }
  char page[400];
  snprintf(page, sizeof(page),
           "<html><head><meta name=viewport content='width=device-width,initial-scale=1'></head>"
           "<body style='font:18px sans-serif;background:#111;color:#eee;padding:16px'><h2>%s</h2>"
           "<p><a style='color:#6cf' href=/airspace>Back</a></p></body></html>", msg);
  wifiLogServer.send(200, "text/html; charset=utf-8", page);
}


// ========== WAYPOINTS: elenco dei campi da file XCSoar (.cup) caricato dalla pagina web ==========
// Il file .cup (CSV: name,code,country,lat,lon,elev,style,rwdir,rwlen,freq,...) viene convertito in /wpt.bin:
// solo i campi di volo (style 2 erba, 4 volo a vela, 5 pista dura). All'avvio /wpt.bin, se c'e', sostituisce
// l'elenco compilato nel firmware. "Use built-in list" lo cancella e torna all'elenco del firmware.
// Sito ufficiale dei waypoint XCSoar (per nazione) e file dell'Italia
#define WPT_LINK_XCSOAR "https://download.xcsoar.org/content/waypoint/country/"
#define WPT_LINK_XCSOAR_IT "https://download.xcsoar.org/content/waypoint/country/IT-WPT-National-XCSoar.cup"
#define WPT_BIN_PATH "/wpt.bin"
#define WPT_TXT_PATH "/wpt_upload.cup"
static const uint32_t WPT_MAGIC = 0x32545057UL;   // "WPT2" (con la sigla del campo)
static const uint32_t WPT_MAGIC_V1 = 0x31545057UL;   // "WPT1": file vecchio, senza sigla
static const int WPT_MAX = 12000;
static const int WPT_NAME_LEN = 26;
struct WptDisk { int32_t lat5, lon5; uint8_t rwy, hdgOfs; uint16_t lenM, freqK; char name[WPT_NAME_LEN]; char code[8]; };   // hdgOfs: vedi airfieldRwyDeg
static const size_t WPT_DISK_V1_SIZE = offsetof(WptDisk, code);   // record dei file "WPT1"
static const int WPT_CODE_LEN = 8;
static AirfieldRec *wptRecs = nullptr;
static char *wptNames = nullptr;
static char *wptCodes = nullptr;   // sigla del campo (colonna code del .cup), solo per l'elenco da file
static char wptFileInfo[72] = "Built-in list";
static bool wptUploadOk = false;
static char wptUploadErr[64] = "";
static char wptUploadName[40] = "";
static File wptUploadFile;

// nome del campo: va bene sia "const char *name" sia "char name[N]"
// (dentro una struct: l'IDE Arduino non genera prototipi per i suoi metodi, che con i template non funzionano)
struct WptName {
  static void set(const char *&d, const char *src) { d = src; }
  template <size_t N> static void set(char (&d)[N], const char *src) { strncpy(d, src, N - 1); d[N - 1] = '\0'; }
};

static void wptFree() {
  if (wptRecs) { heap_caps_free(wptRecs); wptRecs = nullptr; }
  if (wptNames) { heap_caps_free(wptNames); wptNames = nullptr; }
  if (wptCodes) { heap_caps_free(wptCodes); wptCodes = nullptr; }
  if (gAirfieldHdgOfs) { heap_caps_free(gAirfieldHdgOfs); gAirfieldHdgOfs = nullptr; }
}

static const char *airfieldCode(int i) {
  if (wptCodes && gAirfields == wptRecs && i >= 0 && i < gAirfieldCount) return wptCodes + (size_t)i * WPT_CODE_LEN;
  return "";
}
// Carica /wpt.bin; senza file (o se non valido) torna all'elenco del firmware.
bool waypointsLoad() {
  if (navGotoActive) navCancelGoto();   // gli indici dei campi cambiano: la destinazione GO TO non vale piu'
  gAirfields = AIRFIELDS_FIRMWARE;      // prima di liberare la memoria del vecchio elenco
  gAirfieldCount = AIRFIELDS_FIRMWARE_COUNT;
  wptFree();
  bool ok = false;
  int n = 0;
  if (trackFsReady && LittleFS.exists(WPT_BIN_PATH)) {
    File fl = LittleFS.open(WPT_BIN_PATH, FILE_READ);
    uint32_t magic = 0, cnt = 0;
    if (fl && fl.read((uint8_t *)&magic, 4) == 4 && fl.read((uint8_t *)&cnt, 4) == 4 &&
        (magic == WPT_MAGIC || magic == WPT_MAGIC_V1) && cnt > 0 && cnt <= (uint32_t)WPT_MAX) {
      const size_t recSize = magic == WPT_MAGIC ? sizeof(WptDisk) : WPT_DISK_V1_SIZE;
      wptRecs = (AirfieldRec *)heap_caps_calloc(cnt, sizeof(AirfieldRec), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      wptNames = (char *)heap_caps_calloc(cnt, WPT_NAME_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      wptCodes = (char *)heap_caps_calloc(cnt, WPT_CODE_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      gAirfieldHdgOfs = (uint8_t *)heap_caps_calloc(cnt, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (wptRecs && wptNames && wptCodes && gAirfieldHdgOfs) {
        WptDisk d;
        for (uint32_t i = 0; i < cnt; ++i) {
          memset(&d, 0, sizeof(d));
          if (fl.read((uint8_t *)&d, recSize) != recSize) break;
          char *cd = wptCodes + (size_t)n * WPT_CODE_LEN;
          memcpy(cd, d.code, WPT_CODE_LEN); cd[WPT_CODE_LEN - 1] = '\0';
          AirfieldRec &r = wptRecs[n];
          memset((void *)&r, 0, sizeof(r));
          char *nm = wptNames + (size_t)n * WPT_NAME_LEN;
          memcpy(nm, d.name, WPT_NAME_LEN); nm[WPT_NAME_LEN - 1] = '\0';
          WptName::set(r.name, nm);
          r.lat5 = (decltype(r.lat5))d.lat5;
          r.lon5 = (decltype(r.lon5))d.lon5;
          r.rwy = (decltype(r.rwy))d.rwy;
          gAirfieldHdgOfs[n] = d.hdgOfs;   // 0 nei file caricati con il firmware vecchio
          r.rwLenM = (decltype(r.rwLenM))d.lenM;
          r.freqK = (decltype(r.freqK))d.freqK;
          ++n;
        }
        ok = n > 0;
      }
    }
    if (fl) fl.close();
  }
  if (ok) {
    gAirfields = wptRecs;
    gAirfieldCount = n;
    snprintf(wptFileInfo, sizeof(wptFileInfo), "File (%d fields)", n);
    if (preferences.begin("asp", true)) {   // nome del file caricato, se salvato
      String sv = preferences.getString("wptname", "");
      preferences.end();
      if (sv.length() > 0) snprintf(wptFileInfo, sizeof(wptFileInfo), "%s", sv.c_str());
    }
  } else {
    wptFree();
    gAirfields = AIRFIELDS_FIRMWARE;
    gAirfieldCount = AIRFIELDS_FIRMWARE_COUNT;
    snprintf(wptFileInfo, sizeof(wptFileInfo), "Built-in list (%d fields)", gAirfieldCount);
  }
  ++gAirfieldsGen;
  AVEA_LOGF("WPT: %d campi (%s)\n", gAirfieldCount, ok ? "file" : "firmware");
  return ok;
}

// "4542.781N" / "00841.954E" -> gradi * 1e5
static bool wptParseCoord(const char *s, bool isLat, int32_t &out) {
  if (!s || !*s) return false;
  const int dd = isLat ? 2 : 3;
  const size_t L = strlen(s);
  if (L < (size_t)dd + 3) return false;
  char hemi = s[L - 1];
  char degs[4] = {0};
  memcpy(degs, s, dd);
  const float deg = atof(degs), mins = atof(s + dd);
  if (mins < 0 || mins >= 60.0f) return false;
  double v = deg + mins / 60.0;
  if (hemi == 'S' || hemi == 's' || hemi == 'W' || hemi == 'w') v = -v;
  else if (!(hemi == 'N' || hemi == 'n' || hemi == 'E' || hemi == 'e')) return false;
  out = (int32_t)lround(v * 100000.0);
  return true;
}
// "800m", "800.0m", "1.2km", "2600ft", "0.5nm" -> metri
static int wptParseLen(const char *s) {
  if (!s || !*s) return 0;
  const float v = atof(s);
  const char *u = s;
  while (*u && (isdigit((unsigned char)*u) || *u == '.' || *u == ' ')) ++u;
  float m = v;
  if (!strncasecmp(u, "km", 2)) m = v * 1000.0f;
  else if (!strncasecmp(u, "ft", 2)) m = v * 0.3048f;
  else if (!strncasecmp(u, "nm", 2)) m = v * 1852.0f;
  else if (!strncasecmp(u, "ml", 2)) m = v * 1609.0f;
  return constrain((int)lroundf(m), 0, 9999);
}
// divide una riga CSV (con virgolette) in campi; ritorna il numero di campi
static int wptSplitCsv(char *line, char **f, int maxF) {
  int n = 0;
  char *p = line;
  while (*p && n < maxF) {
    while (*p == ' ') ++p;
    if (*p == '"') {
      ++p; f[n++] = p;
      char *w = p;
      while (*p) {
        if (*p == '"' && p[1] == '"') { *w++ = '"'; p += 2; continue; }
        if (*p == '"') { ++p; break; }
        *w++ = *p++;
      }
      *w = '\0';
      while (*p && *p != ',') ++p;
      if (*p == ',') ++p;
    } else {
      f[n++] = p;
      while (*p && *p != ',') ++p;
      if (*p == ',') { *p = '\0'; ++p; }
    }
  }
  return n;
}
// Converte il file .cup caricato in /wpt.bin; ritorna i campi salvati (0 = errore, con messaggio)
static int waypointsConvert(char *err, size_t errSize) {
  File in = LittleFS.open(WPT_TXT_PATH, FILE_READ);
  if (!in) { snprintf(err, errSize, "Cannot read uploaded file"); return 0; }
  File out = LittleFS.open("/wpt.tmp", FILE_WRITE);
  if (!out) { in.close(); snprintf(err, errSize, "Cannot write to internal memory"); return 0; }
  uint32_t magic = WPT_MAGIC, cnt = 0;
  out.write((const uint8_t *)&magic, 4); out.write((const uint8_t *)&cnt, 4);
  int cName = 0, cCode = 1, cLat = 3, cLon = 4, cStyle = 6, cDir = 7, cLen = 8, cFreq = 9;   // colonne di serie
  static char line[400];
  char *fld[20];
  bool first = true;
  while (in.available()) {
    size_t L = in.readBytesUntil('\n', line, sizeof(line) - 1);
    line[L] = '\0';
    if (L > 0 && line[L - 1] == '\r') line[--L] = '\0';
    if (first && L >= 3 && (uint8_t)line[0] == 0xEF && (uint8_t)line[1] == 0xBB && (uint8_t)line[2] == 0xBF) memmove(line, line + 3, L - 2);   // BOM
    if (strstr(line, "-----Related Tasks-----")) break;
    if (L == 0) continue;
    const int nf = wptSplitCsv(line, fld, 20);
    if (first) {
      first = false;
      if (nf > 4 && !strcasecmp(fld[0], "name")) {   // intestazione: posizione delle colonne
        cFreq = -1; cDir = -1; cLen = -1; cCode = -1;
        for (int k = 0; k < nf; ++k) {
          if (!strcasecmp(fld[k], "lat")) cLat = k;
          else if (!strcasecmp(fld[k], "lon")) cLon = k;
          else if (!strcasecmp(fld[k], "style")) cStyle = k;
          else if (!strcasecmp(fld[k], "rwdir")) cDir = k;
          else if (!strcasecmp(fld[k], "rwlen")) cLen = k;
          else if (!strcasecmp(fld[k], "freq")) cFreq = k;
          else if (!strcasecmp(fld[k], "name")) cName = k;
          else if (!strcasecmp(fld[k], "code")) cCode = k;
        }
        continue;
      }
    }
    if (nf <= cLon || nf <= cStyle) continue;
    const int style = atoi(fld[cStyle]);
    if (style != 2 && style != 4 && style != 5) continue;   // solo campi di volo
    WptDisk d;
    memset(&d, 0, sizeof(d));
    int32_t la, lo;
    if (!wptParseCoord(fld[cLat], true, la) || !wptParseCoord(fld[cLon], false, lo)) continue;
    d.lat5 = la; d.lon5 = lo;
    int dir = (cDir >= 0 && cDir < nf) ? atoi(fld[cDir]) : 0;
    if (cDir >= 0 && cDir < nf) {   // "06", "02": numero della pista scritto al posto dei gradi
      const char *ds = fld[cDir];
      while (*ds == ' ' || *ds == '"') ++ds;
      if (ds[0] == '0' && ds[1] >= '1' && ds[1] <= '9' && (ds[2] == '\0' || ds[2] == '"' || ds[2] == ' ')) dir = (ds[1] - '0') * 10;
    }
    d.lenM = (uint16_t)((cLen >= 0 && cLen < nf) ? wptParseLen(fld[cLen]) : 0);
    if (dir > 0 || d.lenM > 0) { int r = ((dir % 360) + 5) / 10; if (r <= 0) r = 36; if (r > 36) r -= 36; d.rwy = (uint8_t)r; }
    if (dir > 0 && d.rwy > 0) {   // direzione vera: scarto dal numero della pista x 10 (di solito da -5 a +4 gradi)
      int ofs = (dir % 360) - d.rwy * 10;
      while (ofs > 180) ofs -= 360;
      while (ofs < -180) ofs += 360;
      if (ofs >= -19 && ofs <= 19) d.hdgOfs = (uint8_t)(ofs + 20);
    }
    if (cFreq >= 0 && cFreq < nf && fld[cFreq][0]) {
      const long k = lround(atof(fld[cFreq]) * 1000.0);   // kHz
      if (k >= 100000 && k <= 165535) d.freqK = (uint16_t)(k - 100000);
    }
    strncpy(d.name, fld[cName], WPT_NAME_LEN - 1);
    if (cCode >= 0 && cCode < nf) strncpy(d.code, fld[cCode], WPT_CODE_LEN - 1);   // sigla, es. LIML
    out.write((const uint8_t *)&d, sizeof(d));
    if (++cnt >= (uint32_t)WPT_MAX) break;
    if ((cnt & 63) == 0) esp_task_wdt_reset();
  }
  in.close();
  out.seek(4); out.write((const uint8_t *)&cnt, 4);
  out.close();
  if (cnt == 0) { LittleFS.remove("/wpt.tmp"); snprintf(err, errSize, "No airfields found (is it a .cup file?)"); return 0; }
  LittleFS.remove(WPT_BIN_PATH);
  LittleFS.rename("/wpt.tmp", WPT_BIN_PATH);
  LittleFS.remove(WPT_TXT_PATH);
  return (int)cnt;
}

static void wifiWaypointsPage() {
  if (wifiLogServer.hasArg("builtin")) {
    LittleFS.remove(WPT_BIN_PATH);
    waypointsLoad();
  }
  static char buf[2000];
  snprintf(buf, sizeof(buf),
    "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Waypoints</title><style>body{font:17px sans-serif;max-width:540px;margin:20px auto;padding:0 14px;background:#111;color:#eee}.bl{color:#fd4;font-weight:bold;font-size:15px;animation:bl 1s steps(2,start) infinite}@keyframes bl{to{visibility:hidden}}"
    ".c{background:#222;border-radius:10px;padding:12px;margin:12px 0}input,button{font:17px sans-serif;margin:8px 0}"
    "button{padding:10px 16px;background:#1685ff;color:#fff;border:0;border-radius:8px}a{color:#6cf}"
    "a.l{display:block;margin:8px 0;padding:10px 12px;background:#333;border-radius:8px;text-decoration:none}</style></head><body>"
    "<h2>Waypoints</h2><div class=c><b>In use:</b> %s</div>"
    "<div class=c><form method=POST action=/waypoints enctype=multipart/form-data>"
    "<p>Updated airfield list: XCSoar waypoint file (.cup). Only airfields are kept "
    "(NEAREST FIELDS, FIELDS, GO TO). Upload on the ground.</p>"
    "<input type=file name=wpt accept='.cup,.txt,.csv' required><br>"
    "<a class=l href='" WPT_LINK_XCSOAR_IT "'>&#11015; Italy, official XCSoar file (IT-WPT-National-XCSoar.cup)</a>"
    "<a class=l target=_blank rel=noopener href='" WPT_LINK_XCSOAR "'>&#127760; XCSoar waypoints, all countries</a>"
    "<a class=l target=_blank rel=noopener href='" ASP_LINK_DRIVE "'>&#128193; Shared folder (Google Drive)</a>"
    "<p class=bl>Download the file to the phone from one of the links, then choose it above.</p>"
    "<p class=bl>Load the same file on master and slave.</p>"
    "<button type=submit>Upload waypoints</button></form></div>"
    "<p><a href=/waypoints?builtin=1 onclick=\"return confirm('Use the built-in list?')\">Use built-in list</a></p>"
    "<p><a href=/>&larr; Flight logs</a></p>"
    "<p style='color:#aaa;font-size:14px'>Some fields may be closed, private or out of date: always check before use.</p>"
    "</body></html>", wptFileInfo);
  wifiLogServer.send(200, "text/html; charset=utf-8", buf);
}
static void wifiWaypointsUpload() {
  HTTPUpload &up = wifiLogServer.upload();
  if (up.status == UPLOAD_FILE_START) {
    wptUploadOk = false;
    wptUploadErr[0] = '\0';
    strncpy(wptUploadName, up.filename.c_str(), sizeof(wptUploadName) - 1);
    wptUploadName[sizeof(wptUploadName) - 1] = '\0';
    if (flightTimerRunning || automaticFlightRunning) { strcpy(wptUploadErr, "Not available during flight"); return; }
    if (!trackFsReady) { strcpy(wptUploadErr, "Internal memory not available"); return; }
    wptUploadFile = LittleFS.open(WPT_TXT_PATH, FILE_WRITE);
    if (!wptUploadFile) strcpy(wptUploadErr, "Cannot write to internal memory");
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (wptUploadFile && wptUploadErr[0] == '\0') {
      if (wptUploadFile.write(up.buf, up.currentSize) != up.currentSize) strcpy(wptUploadErr, "Memory full");
      esp_task_wdt_reset();
    }
  } else if (up.status == UPLOAD_FILE_END) {
    if (wptUploadFile) wptUploadFile.close();
    wptUploadOk = wptUploadErr[0] == '\0';
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (wptUploadFile) wptUploadFile.close();
    strcpy(wptUploadErr, "Upload interrupted");
  }
}
static void wifiWaypointsFinish() {
  char msg[160];
  if (!wptUploadOk) {
    snprintf(msg, sizeof(msg), "Upload failed: %s", wptUploadErr[0] ? wptUploadErr : "unknown error");
  } else {
    char err[64] = "";
    const int n = waypointsConvert(err, sizeof(err));
    if (n > 0) {
      waypointsLoad();
      snprintf(wptFileInfo, sizeof(wptFileInfo), "%.36s (%d fields)", wptUploadName, gAirfieldCount);
      if (preferences.begin("asp", false)) { preferences.putString("wptname", wptFileInfo); preferences.end(); }
      snprintf(msg, sizeof(msg), "Loaded %d airfields", n);
    } else snprintf(msg, sizeof(msg), "Conversion failed: %s", err);
  }
  char page[400];
  snprintf(page, sizeof(page),
           "<html><head><meta name=viewport content='width=device-width,initial-scale=1'></head>"
           "<body style='font:18px sans-serif;background:#111;color:#eee;padding:16px'><h2>%s</h2>"
           "<p><a style='color:#6cf' href=/waypoints>Back</a></p></body></html>", msg);
  wifiLogServer.send(200, "text/html; charset=utf-8", page);
}

static bool wifiLogEndEnvValid(const FlightLog &log) {
  return log.endEnvironmentValid && isfinite(log.endTemp) && isfinite(log.endHumidity) &&
         log.endHumidity >= 0.0f && log.endHumidity <= 100.0f;
}

static void wifiLogSendPage() {
  const int count = min(getNumSavedLogs(), MAX_LOGS);
  wifiLogServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  wifiLogServer.send(200, "text/html; charset=utf-8", "");
  wifiLogServer.sendContent(F(
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width,initial-scale=1'>"
    "<title>Vario - Flight Logs</title><style>"
    "body{font-family:sans-serif;margin:12px;background:#111;color:#eee}"
    "h1{font-size:20px}.c{background:#222;border-radius:10px;padding:10px;margin:10px 0}"
    ".c h2{font-size:16px;margin:0 0 6px;color:#6cf}td{padding:2px 12px 2px 0}"
    "td:first-child{color:#8d8}a.b{display:inline-block;background:#06c;color:#fff;"
    "padding:10px 16px;border-radius:8px;text-decoration:none}"
    "svg.i{width:16px;height:16px;vertical-align:-3px;margin-right:6px;fill:#fd4;stroke:#fd4}</style></head><body>"
    // icone delle voci (definite una volta, usate in ogni log)
    "<svg width=0 height=0 style=position:absolute><defs>"
    "<symbol id=i0 viewBox='-8 -8 16 16'><circle r=6 fill=none stroke-width=1.6 /><path d='M0 0V-4M0 0L3 1' fill=none stroke-width=1.4 stroke-linecap=round /></symbol>"
    "<symbol id=i1 viewBox='-8 -8 16 16'><circle cx=3 cy=-3 r=4 stroke=none /><path d='M-1 -2H7L3 5Z' stroke=none /><circle cx=3 cy=-3 r=1.2 fill=#222 stroke=none /><circle cx=-6 cy=6 r=1 stroke=none /><circle cx=-3 cy=6 r=1 stroke=none /><circle cx=0 cy=6 r=1 stroke=none /></symbol>"
    "<symbol id=i2 viewBox='-8 -8 16 16'><rect x=-6 y=-6 width=8 height=13 rx=1 stroke=none /><rect x=-4 y=-4 width=4 height=3 fill=#222 stroke=none /><path d='M2 -4L5 -2V4H3' fill=none stroke-width=1.3 /></symbol>"
    "<symbol id=i3 viewBox='-8 -8 16 16'><path d='M-6 0H6' stroke-width=2.4 stroke-linecap=round /><path d='M2 0L-2 -7L-3 0ZM2 0L-2 7L-3 0ZM-4 0L-6 -3L-7 0ZM-4 0L-6 3L-7 0Z' stroke=none /></symbol>"
    "<symbol id=i4 viewBox='-8 -8 16 16'><path d='M-7 6L-1 -5L5 6ZM-1 6L3 -1L7 6Z' stroke=none /></symbol>"
    "<symbol id=i5 viewBox='-8 -8 16 16'><path d='M-7 5L-3 1L0 3L5 -3' fill=none stroke-width=1.7 stroke-linecap=round /><path d='M7 -6L1 -4L6 0Z' stroke=none /></symbol>"
    "<symbol id=i6 viewBox='-8 -8 16 16'><rect x=-2 y=-7 width=5 height=10 rx=2 stroke=none /><circle cy=4 r=3.5 stroke=none /><path d='M4 -5H6M4 -2H6M4 1H6' stroke-width=1 /></symbol>"
    "<symbol id=i7 viewBox='-8 -8 16 16'><circle cy=2 r=4.5 stroke=none /><path d='M-4.3 1H4.3L0 -7Z' stroke=none /></symbol>"
    "<symbol id=i8 viewBox='-8 -8 16 16'><g transform='rotate(-45)'><rect x=-2 y=-3 width=4 height=6 stroke=none /><rect x=-7.5 y=-2 width=4.5 height=4 stroke=none /><rect x=3 y=-2 width=4.5 height=4 stroke=none /></g><path d='M3 6A4 4 0 0 0 6 3M3 8.5A6.5 6.5 0 0 0 8.5 3' fill=none stroke-width=1.1 /></symbol>"
    "</defs></svg>"
    "<h1>Recent Flight Logs</h1>"
    "<p><a class=b href=/airspace>Airspace</a> <a class=b href=/waypoints>Waypoints</a></p>"));   // Update Firmware tolto dalla pagina (resta l'indirizzo /update)
  if (count == 0) {
    wifiLogServer.sendContent(F("<p>No saved flight logs.</p></body></html>"));
    wifiLogServer.sendContent("");
    return;
  }
  wifiLogServer.sendContent(F("<p><a class=b href=/logs.csv download=logs.csv>Download report</a></p>"));
  char buf[1000];
  for (int i = 0; i < count; ++i) {
    FlightLog l;
    if (!readLogFromPrefs(i, l)) continue;
    const unsigned long sec = l.durationSec;
    char env[48] = "-";
    char hum[32] = "-";
    if (wifiLogEndEnvValid(l)) {
      snprintf(env, sizeof(env), "%.0f &deg;C", l.endTemp);
      snprintf(hum, sizeof(hum), "%.0f %%", l.endHumidity);
    }
    char fuelTxt[16];
    if (l.fuelUsed >= 0.0f && isfinite(l.fuelUsed)) snprintf(fuelTxt, sizeof(fuelTxt), "%.1f l", l.fuelUsed);
    else strcpy(fuelTxt, "-");
    snprintf(buf, sizeof(buf),
      "<div class=c><h2>#%d &ndash; %s</h2><table>"
      "<tr><td><svg class=i><use href=#i8 /></svg>Time<td>%.5s &rarr; %.5s"
      "<tr><td><svg class=i><use href=#i0 /></svg>Duration<td>%02lu:%02lu:%02lu"
      "<tr><td><svg class=i><use href=#i1 /></svg>Distance<td>%.1f km"
      "<tr><td><svg class=i><use href=#i2 /></svg>Approx Fuel<td>%s"
      "<tr><td><svg class=i><use href=#i3 /></svg>Max GPS speed<td>%.0f km/h"
      "<tr><td><svg class=i><use href=#i4 /></svg>Max altitude<td>%.0f m"
      "<tr><td><svg class=i><use href=#i5 /></svg>Max climb<td>%.1f m/s"
      "<tr><td><svg class=i><use href=#i6 /></svg>Temperature (end)<td>%s"
      "<tr><td><svg class=i><use href=#i7 /></svg>Humidity (end)<td>%s</table>",
      i + 1, l.date, l.startTime, l.stopTime,
      sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL,
      l.distance, fuelTxt, l.maxSpeed, l.maxAltitude, l.maxClimb,
      env, hum);
    wifiLogServer.sendContent(buf);
    if (trackExistsForLog(l)) {
      snprintf(buf, sizeof(buf), "<p><a class=b href=/track.kml?i=%d>Track (Google Earth)</a></p>", i);
      wifiLogServer.sendContent(buf);
    }
    wifiLogServer.sendContent("</div>");
    esp_task_wdt_reset();
  }
  wifiLogServer.sendContent(F("</body></html>"));
  wifiLogServer.sendContent("");
}

static void wifiLogSendCsv() {
  const int count = min(getNumSavedLogs(), MAX_LOGS);
  wifiLogServer.sendHeader("Content-Disposition", "attachment; filename=\"logs.csv\"");
  wifiLogServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  wifiLogServer.send(200, "text/csv; charset=utf-8", "");
  wifiLogServer.sendContent(F("Field,Value\r\n"));
  char row[240];
  for (int i = 0; i < count; ++i) {
    FlightLog l;
    if (!readLogFromPrefs(i, l)) continue;
    const unsigned long sec = l.durationSec;
    char env[48] = "-";
    char hum[32] = "-";
    if (wifiLogEndEnvValid(l)) {
      snprintf(env, sizeof(env), "%.0f C", l.endTemp);
      snprintf(hum, sizeof(hum), "%.0f %%", l.endHumidity);
    }

    // Intestazione della scheda/log (una riga vuota sopra tranne il primo)
    if (i > 0) wifiLogServer.sendContent("\r\n");
    snprintf(row, sizeof(row), "Log #%d - %s,\r\n", i + 1, l.date);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Time,%.5s -> %.5s\r\n", l.startTime, l.stopTime);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Duration,%02lu:%02lu:%02lu\r\n",
             sec / 3600UL, (sec / 60UL) % 60UL, sec % 60UL);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Distance,%.1f km\r\n", l.distance);
    wifiLogServer.sendContent(row);

    // FUEL ESTIMATE al posto di Max sink, subito dopo Distance
    if (l.fuelUsed >= 0.0f && isfinite(l.fuelUsed)) snprintf(row, sizeof(row), "Approx Fuel,%.1f l\r\n", l.fuelUsed);
    else snprintf(row, sizeof(row), "Approx Fuel,-\r\n");
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Max GPS speed,%.0f km/h\r\n", l.maxSpeed);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Max altitude,%.0f m\r\n", l.maxAltitude);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Max climb,%.1f m/s\r\n", l.maxClimb);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Temperature (end),%s\r\n", env);
    wifiLogServer.sendContent(row);

    snprintf(row, sizeof(row), "Humidity (end),%s\r\n", hum);
    wifiLogServer.sendContent(row);

    esp_task_wdt_reset();
  }
  wifiLogServer.sendContent("");
}

static void setWifiOtaError(const char *message) {
  wifiOtaUploadFailed = true;
  snprintf(wifiOtaErrorMessage, sizeof(wifiOtaErrorMessage), "%s",
           message != nullptr ? message : "unknown error");
  AVEA_LOGF("Wi-Fi OTA: %s\n", wifiOtaErrorMessage);
}

static void wifiOtaShowPage() {
  wifiLogServer.send(200, "text/html; charset=utf-8", F(
    "<!doctype html><html><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>Firmware Update</title><style>"
    "body{font:18px sans-serif;max-width:540px;margin:32px auto;padding:0 16px;background:#111;color:#eee}"
    "input,button{font:18px sans-serif;margin:12px 0}button{padding:12px 18px;background:#1685ff;color:white;border:0;border-radius:8px}"
    "</style></head><body><h2>Firmware Update</h2>"
    "<p>Select the <b>.bin</b> file saved on your phone. Keep the board powered during upload.</p>"
    "<form method='POST' action='/update' enctype='multipart/form-data'>"
    "<input type='file' name='firmware' accept='.bin,application/octet-stream' required><br>"
    "<button type='submit'>Upload and Update</button></form>"
    "<p>The ESP32 will restart when the update is complete.</p></body></html>"));
}

static void wifiOtaHandleUpload() {
  HTTPUpload &upload = wifiLogServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    wifiOtaUploadActive = false;
    wifiOtaUploadFailed = false;
    wifiOtaUploadSucceeded = false;
    wifiOtaErrorMessage[0] = '\0';

    if (flightTimerRunning || automaticFlightRunning || wifiLogState != WIFI_LOG_CONNECTED) {
      setWifiOtaError("Update unavailable during flight or without Wi-Fi");
    } else if (!upload.filename.endsWith(".bin")) {
      setWifiOtaError("Select a .bin file");
    } else if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      setWifiOtaError(Update.errorString());
    } else {
      wifiOtaUploadActive = true;
      AVEA_LOGF("Wi-Fi OTA: upload di %s\n", upload.filename.c_str());
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (wifiOtaUploadActive && !wifiOtaUploadFailed) {
      const size_t written = Update.write(upload.buf, upload.currentSize);
      if (written != upload.currentSize) {
        setWifiOtaError(Update.errorString());
        Update.abort();
        wifiOtaUploadActive = false;
      } else {
        esp_task_wdt_reset();
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (wifiOtaUploadActive && !wifiOtaUploadFailed) {
      if (Update.end(true)) {
        wifiOtaUploadSucceeded = true;
        AVEA_LOGF("Wi-Fi OTA: immagine verificata (%u byte)\n", (unsigned)upload.totalSize);
      } else {
        setWifiOtaError(Update.errorString());
      }
      wifiOtaUploadActive = false;
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (wifiOtaUploadActive) Update.abort();
    wifiOtaUploadActive = false;
    setWifiOtaError("Upload interrupted");
  }
}

static void wifiOtaFinishRequest() {
  if (!wifiOtaUploadSucceeded || wifiOtaUploadFailed) {
    String response = "Update failed";
    if (wifiOtaErrorMessage[0] != '\0') response += String(": ") + wifiOtaErrorMessage;
    wifiLogServer.send(500, "text/plain; charset=utf-8", response);
    return;
  }
  wifiLogServer.send(200, "text/html; charset=utf-8",
                     "<html><body><h2>Update complete</h2><p>Restarting the board...</p></body></html>");
  delay(700);
  ESP.restart();
}

// ========== AGGIORNAMENTO AUTOMATICO DA GITHUB ==========
// Quando il WiFi si collega all'hotspot (pagina WIFI LOG) lo strumento controlla
// l'ultimo commit del file OTA_GH_FILE nel repository OTA_GH_REPO e scarica il
// firmware SOLO SE: (1) il MAC del dispositivo e' nel file OTA_GH_MACS dello stesso repository e
// (2) il file su GitHub e' piu' recente del firmware installato.
// "Piu' recente" = data del commit > max(data di compilazione di questo firmware,
// data del commit dell'ultimo aggiornamento installato da GitHub).
// Il repository deve essere pubblico. I MAC si leggono dalla pagina FIRMWARE (sotto VERSION).
// Se macs.txt non e' raggiungibile o il MAC manca, nessun aggiornamento.
static const char* const OTA_GH_REPO = "antoniolaser/AVEA";
static const char* const OTA_GH_FILE = "sketch.ino.bin";
static const char* const OTA_GH_MACS = "macs.txt";  // un MAC per riga, righe con # = commenti
static const uint32_t OTA_MIN_FREE_HEAP = 60000UL;  // RAM interna minima per il TLS
static const unsigned long OTA_STALL_TIMEOUT_MS = 15000UL;

static char otaStatusText[34] = "";
static uint16_t otaStatusColor = TFT_WHITE;
static bool otaAutoChecked = false;
static portMUX_TYPE otaMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool otaStatusDirty = false;      // il task ha cambiato lo stato: lo ridisegna il loop
static volatile bool otaTaskBusy = false;         // controllo GitHub in corso (task separato)
static TaskHandle_t otaWorkerHandle = nullptr;
static volatile uint32_t otaSessionId = 0;        // cambia a ogni apertura della pagina WIFI
static uint32_t otaWorkerSession = 0;
static bool otaUpdatePending = false;   // nuova versione trovata: attende il tocco sulla riga di stato
static String otaPendingSha;
static uint32_t otaPendingEpoch = 0;
// Area di tocco (display 360x360) della riga di stato "FW: ..." nella pagina WIFI LOG.
#define OTA_TAP_X_MIN 0
#define OTA_TAP_X_MAX 359
#define OTA_TAP_Y_MIN 238   // allargata verso l'alto: comprende anche la riga "Browser: tap Update firmware"
#define OTA_TAP_Y_MAX 359   // fino al bordo inferiore del display

void otaGetMac(char *out) {
  uint8_t m[6] = {0};
  esp_read_mac(m, ESP_MAC_WIFI_STA);  // legge l'eFuse: funziona anche con il WiFi spento
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

static int64_t otaDaysFromCivil(int y, unsigned m, unsigned d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m > 2 ? m - 3 : m + 9) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return (int64_t)era * 146097 + (int64_t)doe - 719468;
}

static int64_t otaEpochUtc(int y, int mo, int d, int h, int mi, int s) {
  return otaDaysFromCivil(y, (unsigned)mo, (unsigned)d) * 86400LL + h * 3600LL + mi * 60LL + s;
}

// Ore 01:00 UTC dell'ultima domenica del mese (cambio ora legale europeo).
static int64_t otaLastSundayUtc(int y, int mo) {
  const int64_t dd = otaDaysFromCivil(y, (unsigned)mo, 31);
  const int wd = (int)(((dd + 4) % 7 + 7) % 7);  // 0 = domenica
  return (dd - wd) * 86400LL + 3600LL;
}

// Data di compilazione (ora locale del PC, Italia CET/CEST) -> epoch UTC.
static uint32_t otaBuildEpoch() {
  static const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *date = __DATE__;  // "Oct  7 2026"
  char mon[4] = { date[0], date[1], date[2], 0 };
  const char *p = strstr(months, mon);
  const int mo = p ? (int)(p - months) / 3 + 1 : 1;
  int d = 0, y = 0, h = 0, mi = 0, sec = 0;
  sscanf(date + 4, "%d %d", &d, &y);
  sscanf(__TIME__, "%d:%d:%d", &h, &mi, &sec);
  const int64_t local = otaEpochUtc(y, mo, d, h, mi, sec);
  const int64_t guess = local - 3600;
  const bool dst = guess >= otaLastSundayUtc(y, 3) && guess < otaLastSundayUtc(y, 10);
  return (uint32_t)(local - (dst ? 7200 : 3600));
}

static bool otaParseIso(const String &iso, uint32_t &epoch) {
  int y, mo, d, h, mi, sec;
  if (sscanf(iso.c_str(), "%d-%d-%dT%d:%d:%d", &y, &mo, &d, &h, &mi, &sec) != 6) return false;
  epoch = (uint32_t)otaEpochUtc(y, mo, d, h, mi, sec);
  return true;
}

static void fwDrawChooser();
extern volatile uint8_t fwMode;
static void otaDrawStatus() {
  if (currentScreen != WIFI_LOGS_SCREEN || wifiLogState != WIFI_LOG_CONNECTED) return;
  if (fwMode != 0) { fwDrawChooser(); return; }   // CHOOSE FW aperto: la pagina mostra l'elenco o l'avanzamento
  char text[sizeof(otaStatusText)];
  uint16_t color;
  portENTER_CRITICAL(&otaMux);
  memcpy(text, otaStatusText, sizeof(text));
  color = otaStatusColor;
  portEXIT_CRITICAL(&otaMux);
  lcd.fillRect(50, 277, 260, 26, themeBg());
  if (text[0] == '\0') return;
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setVividTextColor(color);
  lcd.drawString(text, 180, 290);
}

// Dal task GitHub NON si disegna (il display non e' thread-safe): si aggiorna il testo
// e il loop lo ridisegna. Dal loop (download) si disegna subito.
static void otaSetStatus(const char *msg, uint16_t color) {
  const bool fromWorker = (xTaskGetCurrentTaskHandle() == otaWorkerHandle);
  if (fromWorker && otaWorkerSession != otaSessionId) return;  // sessione WiFi gia' chiusa
  portENTER_CRITICAL(&otaMux);
  snprintf(otaStatusText, sizeof(otaStatusText), "%s", msg);
  otaStatusColor = color;
  portEXIT_CRITICAL(&otaMux);
  AVEA_LOGF("GitHub OTA: %s\n", msg);
  if (fromWorker) otaStatusDirty = true;
  else otaDrawStatus();
}

// Legge macs.txt dal repository. Ritorna 1 = MAC in lista, 0 = non in lista,
// -1 = errore (httpCode contiene il codice HTTP, o <=0 per errore di connessione).
static int otaMacInGithubList(const char *mac, int &httpCode) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setUserAgent("AVEA-vario");
  http.setTimeout(15000);
  const String url = String("https://raw.githubusercontent.com/") + OTA_GH_REPO + "/HEAD/" + OTA_GH_MACS;
  httpCode = 0;
  if (!http.begin(client, url)) return -1;
  httpCode = http.GET();
  if (httpCode != 200) {
    http.end();
    return -1;
  }
  const String body = http.getString();
  http.end();
  int start = 0;
  const int total = (int)body.length();
  while (start < total) {
    int end = body.indexOf('\n', start);
    if (end < 0) end = total;
    String line = body.substring(start, end);
    start = end + 1;
    line.trim();
    if (line.length() == 0 || line[0] == '#') continue;
    if (line.length() > 17) line = line.substring(0, 17);
    if (line.equalsIgnoreCase(mac)) return 1;
  }
  return 0;
}

// Legge sha e data dell'ultimo commit che ha toccato OTA_GH_FILE.
// Il client TLS vive solo qui dentro: la sua RAM viene liberata prima del download.
static bool otaFetchLatest(uint32_t &remoteEpoch, String &sha) {
  WiFiClientSecure client;
  client.setInsecure();  // nessuna verifica del certificato (vedi note)
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setUserAgent("AVEA-vario");
  http.setTimeout(15000);
  const String url = String("https://api.github.com/repos/") + OTA_GH_REPO +
                     "/commits?path=" + OTA_GH_FILE + "&per_page=1";
  if (!http.begin(client, url)) {
    otaSetStatus("FW: connect error", TFT_RED);
    return false;
  }
  http.addHeader("Accept", "application/vnd.github+json");
  const int code = http.GET();
  if (code != 200) {
    char msg[34];
    snprintf(msg, sizeof(msg), "FW: check failed (%d)", code);
    otaSetStatus(msg, TFT_RED);
    http.end();
    return false;
  }
  const String body = http.getString();
  http.end();
  const int iSha = body.indexOf("\"sha\":\"");
  const int iCom = body.indexOf("\"committer\":{");
  const int iDate = iCom >= 0 ? body.indexOf("\"date\":\"", iCom) : -1;
  if (iSha < 0 || iDate < 0) {
    otaSetStatus("FW: file not on GitHub", TFT_RED);
    return false;
  }
  sha = body.substring(iSha + 7, iSha + 7 + 40);
  if (sha.length() != 40 || !otaParseIso(body.substring(iDate + 8, iDate + 8 + 20), remoteEpoch)) {
    otaSetStatus("FW: bad GitHub reply", TFT_RED);
    return false;
  }
  return true;
}

static volatile int fwDlPct = 0;
static volatile uint32_t fwDlDone = 0, fwDlLen = 0;
static String otaUrlEncodePath(const char *p) {   // spazi e caratteri speciali nel nome del file
  String o; char h[4];
  for (; *p; ++p) {
    const unsigned char c = (unsigned char)*p;
    if (isalnum(c) || c == '/' || c == '-' || c == '_' || c == '.' || c == '~') o += (char)c;
    else { snprintf(h, sizeof(h), "%%%02X", c); o += h; }
  }
  return o;
}
static void otaDownloadPath(const String &sha, uint32_t remoteEpoch, const char *path, const char *instName);
static void otaDownloadAndFlash(const String &sha, uint32_t remoteEpoch) { otaDownloadPath(sha, remoteEpoch, OTA_GH_FILE, OTA_GH_FILE); }
// Scarica e installa il file path (nel repository, al commit sha); instName = nome ricordato come "installed"
static void otaDownloadPath(const String &sha, uint32_t remoteEpoch, const char *path, const char *instName) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setUserAgent("AVEA-vario");
  http.setTimeout(15000);
  // URL con lo sha del commit: scarica esattamente il file di cui ho letto la data.
  const String url = String("https://raw.githubusercontent.com/") + OTA_GH_REPO + "/" + sha + "/" + otaUrlEncodePath(path);
  if (!http.begin(client, url)) {
    otaSetStatus("FW: connect error", TFT_RED);
    return;
  }
  const int code = http.GET();
  const int len = http.getSize();
  fwDlPct = 0; fwDlDone = 0; fwDlLen = len > 0 ? (uint32_t)len : 0;
  if (code != 200 || len <= 0) {
    char msg[34];
    snprintf(msg, sizeof(msg), "FW: download failed (%d)", code);
    otaSetStatus(msg, TFT_RED);
    http.end();
    return;
  }
  if (!Update.begin((size_t)len, U_FLASH)) {
    otaSetStatus(Update.getError() == UPDATE_ERROR_SPACE ? "FW: file too large" : "FW: update start failed",
                 TFT_RED);
    http.end();
    return;
  }
  static uint8_t otaBuf[2048];
  WiFiClient *stream = http.getStreamPtr();
  size_t total = 0;
  int lastPct = -1;
  unsigned long lastData = millis();
  bool ok = true;
  while (total < (size_t)len) {
    esp_task_wdt_reset();
    const int avail = stream->available();
    if (avail > 0) {
      const size_t want = (size_t)avail < sizeof(otaBuf) ? (size_t)avail : sizeof(otaBuf);
      const int n = stream->readBytes(otaBuf, want);
      if (n <= 0) continue;
      if (Update.write(otaBuf, (size_t)n) != (size_t)n) { ok = false; break; }
      total += (size_t)n;
      lastData = millis();
      const int pct = (int)((uint64_t)total * 100ULL / (uint64_t)len);
      fwDlPct = pct; fwDlDone = (uint32_t)total;
      if (pct != lastPct && (pct % 2 == 0)) {
        lastPct = pct;
        char msg[34];
        snprintf(msg, sizeof(msg), "FW: updating %d%%", pct);
        otaSetStatus(msg, TFT_YELLOW);
      }
    } else {
      if (!stream->connected()) break;
      if (millis() - lastData > OTA_STALL_TIMEOUT_MS) break;
      delay(2);
    }
  }
  http.end();
  if (!ok || total != (size_t)len) {
    Update.abort();
    otaSetStatus(ok ? "FW: download incomplete" : "FW: flash write failed", TFT_RED);
    return;
  }
  if (!Update.end(true)) {
    Update.abort();
    otaSetStatus("FW: verify failed", TFT_RED);
    return;
  }
  // Ricorda la data del commit installato (namespace "ota": non toccato da "Reset preferences").
  Preferences otaPrefs;
  if (otaPrefs.begin("ota", false)) {
    otaPrefs.putUInt("inst", remoteEpoch);
    otaPrefs.putString("name", instName);   // nome del file installato (lista CHOOSE FW)
    otaPrefs.end();
  }
  otaSetStatus("FW: update complete", TFT_GREEN);
  delay(1000);
  ESP.restart();
}

static void otaCheckWorker(uint32_t session) {
  if (flightTimerRunning || automaticFlightRunning) return;
  char mac[18];
  otaGetMac(mac);
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < OTA_MIN_FREE_HEAP) {
    otaSetStatus("FW: low memory", TFT_RED);
    return;
  }
  otaSetStatus("FW: checking...", TFT_WHITE);
  int listCode = 0;
  const int inList = otaMacInGithubList(mac, listCode);
  if (inList < 0) {
    char msg[34];
    snprintf(msg, sizeof(msg), "FW: MAC list error (%d)", listCode);
    otaSetStatus(msg, TFT_RED);
    return;
  }
  if (inList == 0) {
    AVEA_LOGF("GitHub OTA: MAC %s non in lista\n", mac);
    otaSetStatus("FW: device not enabled", TFT_YELLOW);
    return;
  }
  uint32_t remoteEpoch = 0;
  String sha;
  if (!otaFetchLatest(remoteEpoch, sha)) return;

  uint32_t installed = otaBuildEpoch();
  Preferences otaPrefs;
  if (otaPrefs.begin("ota", true)) {
    const uint32_t stored = otaPrefs.getUInt("inst", 0);
    otaPrefs.end();
    if (stored > installed) installed = stored;
  }
  AVEA_LOGF("GitHub OTA: repo=%u installato=%u sha=%s\n", (unsigned)remoteEpoch, (unsigned)installed, sha.c_str());
  if (remoteEpoch <= installed) {
    otaSetStatus("FW: up to date", TFT_GREEN);
    return;
  }
  if (session != otaSessionId) return;  // la pagina WIFI e' stata chiusa/riaperta nel frattempo
  otaPendingSha = sha;
  otaPendingEpoch = remoteEpoch;
  otaUpdatePending = true;
  otaSetStatus("FW: new! Tap to update", TFT_YELLOW);
}

// Il controllo (TLS + GitHub, alcuni secondi) gira in un task separato: il loop,
// il tocco e gli swipe restano attivi.
static void otaCheckTask(void *arg) {
  otaWorkerSession = (uint32_t)(uintptr_t)arg;
  otaWorkerHandle = xTaskGetCurrentTaskHandle();
  otaCheckWorker(otaWorkerSession);
  otaWorkerHandle = nullptr;
  otaTaskBusy = false;
  vTaskDelete(NULL);
}

static void otaStartCheckTask() {
  if (otaTaskBusy) return;
  otaTaskBusy = true;
  if (xTaskCreatePinnedToCore(otaCheckTask, "otaChk", 16384, (void*)(uintptr_t)otaSessionId,
                              1, nullptr, 0) != pdPASS) {
    otaTaskBusy = false;
    otaSetStatus("FW: task error", TFT_RED);
  }
}

// Chiamata dal tocco sulla riga di stato: scarica e installa la versione trovata.
// Se il download fallisce resta in attesa: un nuovo tocco riprova.
static void otaStartPendingUpdate() {
  if (!otaUpdatePending) return;
  if (flightTimerRunning || automaticFlightRunning) return;
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < OTA_MIN_FREE_HEAP) {
    otaSetStatus("FW: low memory", TFT_RED);
    return;
  }
  otaSetStatus("FW: starting...", TFT_YELLOW);
  otaDownloadAndFlash(otaPendingSha, otaPendingEpoch);
}

static bool otaTapOnStatusLine(int x, int y) {
  return otaUpdatePending && x >= OTA_TAP_X_MIN && x <= OTA_TAP_X_MAX &&
         y >= OTA_TAP_Y_MIN && y <= 303;   // sotto c'e' il pulsante CHOOSE FW
}

// ========== CHOOSE FW: scelta del firmware per nome dalla cartella firmware/ del repository ==========
// WIFI LOG collegato -> pulsante CHOOSE FW -> elenco dei .bin di firmware/ (nome e dimensione, swipe su/giu')
// -> tocco su un file: INSTALL? con data del file -> YES: scarica e installa (anche una versione piu' vecchia).
// Stesse regole dell'aggiornamento automatico: MAC in macs.txt, niente aggiornamenti in volo.
static const char *const FW_DIR = "firmware";
static const int FW_MAX = 20, FW_ROWS = 5, FW_ROW_Y0 = 96, FW_ROW_H = 42;
static const int FW_BTN_X = 110, FW_BTN_Y = 306, FW_BTN_W = 140, FW_BTN_H = 30;
static char fwNames[FW_MAX][48];
static uint32_t fwSizes[FW_MAX];
static int fwCount = 0, fwScroll = 0, fwSel = -1;
volatile uint8_t fwMode = 0;                 // 0 chiuso, 1 elenco, 2 conferma, 3 aggiornamento
static volatile uint8_t fwListState = 0;     // 0 in lettura, 1 pronto, 2 errore
static volatile uint8_t fwInfoState = 0;     // 0 in lettura, 1 pronto, 2 errore
static volatile bool fwDirty = false, fwTaskBusy = false;
static char fwMsg[40] = "";
static String fwSelSha;
static uint32_t fwSelEpoch = 0;
static char fwInstalled[48] = "";

static bool fwGetJson(const String &url, String &body, int &code) {
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setUserAgent("AVEA-vario");
  http.setTimeout(15000);
  code = 0;
  if (!http.begin(client, url)) return false;
  http.addHeader("Accept", "application/vnd.github+json");
  code = http.GET();
  if (code == 200) body = http.getString();
  http.end();
  return code == 200;
}
static void fwListWorker() {
  char mac[18]; otaGetMac(mac);
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < OTA_MIN_FREE_HEAP) { snprintf(fwMsg, sizeof(fwMsg), "Low memory"); fwListState = 2; return; }
  int lc = 0;
  const int inList = otaMacInGithubList(mac, lc);
  if (inList < 0) { snprintf(fwMsg, sizeof(fwMsg), "MAC list error (%d)", lc); fwListState = 2; return; }
  if (inList == 0) { snprintf(fwMsg, sizeof(fwMsg), "Device not enabled"); fwListState = 2; return; }
  String body; int code = 0;
  if (!fwGetJson(String("https://api.github.com/repos/") + OTA_GH_REPO + "/contents/" + FW_DIR, body, code)) {
    if (code == 404) snprintf(fwMsg, sizeof(fwMsg), "No folder firmware/");
    else snprintf(fwMsg, sizeof(fwMsg), "GitHub error (%d)", code);
    fwListState = 2; return;
  }
  int n = 0, pos = 0;
  while (n < FW_MAX) {
    const int iN = body.indexOf("\"name\":\"", pos); if (iN < 0) break;
    const int e = body.indexOf('"', iN + 8); if (e < 0) break;
    String name = body.substring(iN + 8, e);
    const int iS = body.indexOf("\"size\":", e);
    const int iT = body.indexOf("\"type\":\"", e);
    pos = e + 1;
    if (iS < 0 || iT < 0) break;
    const bool isFile = body.startsWith("file", iT + 8);
    String low = name; low.toLowerCase();
    if (!isFile || !low.endsWith(".bin") || name.length() >= sizeof(fwNames[0])) continue;
    snprintf(fwNames[n], sizeof(fwNames[0]), "%s", name.c_str());
    fwSizes[n] = (uint32_t)body.substring(iS + 7, iS + 20).toInt();
    ++n;
  }
  fwCount = n;
  fwListState = 1;
}
static void fwInfoWorker() {
  String body; int code = 0;
  const String path = String(FW_DIR) + "/" + fwNames[fwSel];
  if (!fwGetJson(String("https://api.github.com/repos/") + OTA_GH_REPO + "/commits?path=" + otaUrlEncodePath(path.c_str()) + "&per_page=1", body, code)) {
    snprintf(fwMsg, sizeof(fwMsg), "GitHub error (%d)", code); fwInfoState = 2; return;
  }
  const int iSha = body.indexOf("\"sha\":\"");
  const int iCom = body.indexOf("\"committer\":{");
  const int iDate = iCom >= 0 ? body.indexOf("\"date\":\"", iCom) : -1;
  uint32_t ep = 0;
  if (iSha < 0 || iDate < 0 || !otaParseIso(body.substring(iDate + 8, iDate + 8 + 20), ep)) {
    snprintf(fwMsg, sizeof(fwMsg), "Bad GitHub reply"); fwInfoState = 2; return;
  }
  fwSelSha = body.substring(iSha + 7, iSha + 7 + 40);
  fwSelEpoch = ep;
  fwInfoState = fwSelSha.length() == 40 ? 1 : 2;
}
static void fwTask(void *arg) {
  const int what = (int)(uintptr_t)arg;
  while (otaTaskBusy) vTaskDelay(pdMS_TO_TICKS(200));   // un solo collegamento TLS alla volta
  if (what == 1) fwListWorker(); else fwInfoWorker();
  fwDirty = true;
  fwTaskBusy = false;
  vTaskDelete(NULL);
}
static void fwStartTask(int what) {
  if (fwTaskBusy) return;
  fwTaskBusy = true;
  if (xTaskCreatePinnedToCore(fwTask, "fwTask", 16384, (void *)(uintptr_t)what, 1, nullptr, 0) != pdPASS) {
    fwTaskBusy = false;
    snprintf(fwMsg, sizeof(fwMsg), "Task error");
    if (what == 1) fwListState = 2; else fwInfoState = 2;
  }
}
static void fwOpenChooser() {
  fwMode = 1; fwListState = 0; fwCount = 0; fwScroll = 0; fwSel = -1;
  Preferences pr;
  fwInstalled[0] = '\0';
  if (pr.begin("ota", true)) { String nm = pr.getString("name", ""); pr.end(); snprintf(fwInstalled, sizeof(fwInstalled), "%s", nm.c_str()); }
  fwStartTask(1);
}
static void fwText(const char *t, int x, int y, const lgfx::IFont *f, uint16_t c, decltype(MC_DATUM) datum) {
  lcd.setFreeFont(f);
  lcd.setTextSize(1.0f, 1.0f);
  lcd.setTextDatum(datum);
  setVividTextColor(c);
  lcd.drawString(t, x, y);
}
static void fwFit(const char *in, char *out, size_t outSz, int maxW) {   // accorcia il nome con ".." se troppo largo
  snprintf(out, outSz, "%s", in);
  int len = (int)strlen(out);
  while (len > 4 && lcd.textWidth(out) > maxW) { --len; out[len] = '\0'; out[len - 1] = '.'; out[len - 2] = '.'; }
}
static void fwDrawChooser() {
  lcd.fillScreen(themeBg());
  const lgfx::IFont *f9 = &lgfx::v1::fonts::FreeSansBold9pt7b, *f12 = &lgfx::v1::fonts::FreeSansBold12pt7b,
                    *f18 = &lgfx::v1::fonts::FreeSansBold18pt7b, *f9n = &lgfx::v1::fonts::FreeSans9pt7b;
  char line[64], nm[52];
  if (fwMode == 1) {
    fwText("FIRMWARE", 180, 46, f18, TFT_CYAN, MC_DATUM);
    fwText("GitHub: firmware/", 180, 76, f9n, TFT_LIGHTGREY, MC_DATUM);
    if (fwListState == 0) fwText("Loading...", 180, 180, f12, TFT_WHITE, MC_DATUM);
    else if (fwListState == 2) { fwText(fwMsg, 180, 170, f9, TFT_RED, MC_DATUM); }
    else if (fwCount == 0) fwText("No .bin files", 180, 180, f12, TFT_YELLOW, MC_DATUM);
    else {
      for (int r = 0; r < FW_ROWS && fwScroll + r < fwCount; ++r) {
        const int i = fwScroll + r, y = FW_ROW_Y0 + r * FW_ROW_H;
        const bool inst = fwInstalled[0] && strcmp(fwInstalled, fwNames[i]) == 0;
        const uint16_t frame = inst ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
        lcd.drawRoundRect(58, y, 244, 36, 7, frame); lcd.drawRoundRect(59, y + 1, 242, 34, 6, frame);
        lcd.setFreeFont(f9);
        fwFit(fwNames[i], nm, sizeof(nm), 220);
        fwText(nm, 70, y + 12, f9, TFT_WHITE, ML_DATUM);
        snprintf(line, sizeof(line), "%.2f MB", fwSizes[i] / 1048576.0f);
        fwText(line, 70, y + 27, f9n, TFT_LIGHTGREY, ML_DATUM);
        if (inst) fwText("installed", 292, y + 27, f9n, TFT_GREEN, MR_DATUM);
      }
      if (fwScroll > 0) lcd.fillTriangle(318, 132, 326, 146, 310, 146, themeColor(TFT_LIGHTGREY));
      if (fwScroll + FW_ROWS < fwCount) lcd.fillTriangle(318, 254, 326, 240, 310, 240, themeColor(TFT_LIGHTGREY));
    }
    fwText(fwListState == 1 && fwCount > 0 ? "Tap a file - Swipe to go back" : "Swipe to go back", 180, 322, f9n, TFT_CYAN, MC_DATUM);
  } else if (fwMode == 2) {
    fwText("INSTALL?", 180, 62, f18, TFT_YELLOW, MC_DATUM);
    lcd.setFreeFont(f12);
    fwFit(fwNames[fwSel], nm, sizeof(nm), 280);
    fwText(nm, 180, 108, f12, TFT_WHITE, MC_DATUM);
    if (fwInfoState == 1) {
      time_t t = (time_t)fwSelEpoch; struct tm tmv; gmtime_r(&t, &tmv);
      snprintf(line, sizeof(line), "%04d-%02d-%02d  -  %.2f MB", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, fwSizes[fwSel] / 1048576.0f);
    } else if (fwInfoState == 2) snprintf(line, sizeof(line), "%s", fwMsg);
    else snprintf(line, sizeof(line), "Reading date...");
    fwText(line, 180, 140, f9n, fwInfoState == 2 ? TFT_RED : TFT_LIGHTGREY, MC_DATUM);
    { char ver[32]; firmwareVersionString(ver, sizeof(ver)); snprintf(line, sizeof(line), "Installed: %s", fwInstalled[0] ? fwInstalled : ver); }
    fwText(line, 180, 170, f9n, TFT_LIGHTGREY, MC_DATUM);
    const uint16_t red = themeVivid(TFT_RED), grn = fwInfoState == 1 ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY);
    for (int k = 0; k < 3; ++k) { lcd.drawRoundRect(62 + k, 206 + k, 106 - 2 * k, 56 - 2 * k, 10, red); lcd.drawRoundRect(192 + k, 206 + k, 106 - 2 * k, 56 - 2 * k, 10, grn); }
    fwText("NO", 115, 234, f18, TFT_RED, MC_DATUM);
    lcd.setFreeFont(f18); lcd.setTextDatum(MC_DATUM); lcd.setTextColor(grn, themeBg()); lcd.drawString("YES", 245, 234);
    fwText("Do not switch off", 180, 296, f9, TFT_ORANGE, MC_DATUM);
    fwText("during the update", 180, 316, f9, TFT_ORANGE, MC_DATUM);
  } else {
    fwText("UPDATING", 180, 70, f18, TFT_CYAN, MC_DATUM);
    lcd.setFreeFont(f12);
    fwFit(fwNames[fwSel], nm, sizeof(nm), 280);
    fwText(nm, 180, 120, f12, TFT_WHITE, MC_DATUM);
    char st[34]; uint16_t sc;
    portENTER_CRITICAL(&otaMux); memcpy(st, otaStatusText, sizeof(st)); sc = otaStatusColor; portEXIT_CRITICAL(&otaMux);
    lcd.drawRoundRect(60, 160, 240, 30, 6, themeColor(TFT_WHITE));
    const int w = (int)(234L * fwDlPct / 100L);
    if (w > 0) lcd.fillRoundRect(63, 163, w, 24, 5, themeVivid(TFT_GREEN));
    snprintf(line, sizeof(line), "%d %%", (int)fwDlPct);
    fwText(line, 180, 215, f12, TFT_WHITE, MC_DATUM);
    snprintf(line, sizeof(line), "%.2f / %.2f MB", fwDlDone / 1048576.0f, fwDlLen / 1048576.0f);
    fwText(line, 180, 242, f9n, TFT_LIGHTGREY, MC_DATUM);
    if (sc == TFT_RED) { fwText(st + (strncmp(st, "FW: ", 4) == 0 ? 4 : 0), 180, 282, f9, TFT_RED, MC_DATUM); fwText("Tap to go back", 180, 310, f9n, TFT_CYAN, MC_DATUM); }
    else { fwText("Do not switch off", 180, 290, f9, TFT_ORANGE, MC_DATUM); fwText("Restarts by itself at the end", 180, 312, f9n, TFT_LIGHTGREY, MC_DATUM); }
  }
}
static void fwDrawButton() {
  for (int k = 0; k < 2; ++k) lcd.drawRoundRect(FW_BTN_X + k, FW_BTN_Y + k, FW_BTN_W - 2 * k, FW_BTN_H - 2 * k, 8, themeVivid(TFT_CYAN));
  fwText("CHOOSE FW", 180, FW_BTN_Y + FW_BTN_H / 2, &lgfx::v1::fonts::FreeSansBold9pt7b, TFT_CYAN, MC_DATUM);
}
static bool fwTapOnButton(int x, int y) {
  return fwMode == 0 && wifiLogState == WIFI_LOG_CONNECTED && x >= FW_BTN_X - 10 && x <= FW_BTN_X + FW_BTN_W + 10 && y >= FW_BTN_Y - 4 && y <= 352;
}
// Tocchi e swipe con CHOOSE FW aperto
void fwTouchRelease(bool tap, int dx, int dy, int x, int y) {
  const bool hSwipe = abs(dx) > 50 && abs(dx) > abs(dy);
  const bool vSwipe = abs(dy) > 40 && abs(dy) > abs(dx);
  if (fwMode == 1) {
    if (hSwipe) { fwMode = 0; drawWifiLogsScreen(); playTone(1500, 80); return; }
    if (vSwipe && fwListState == 1) {
      const int maxS = max(0, fwCount - FW_ROWS);
      fwScroll = constrain(fwScroll + (dy < 0 ? 3 : -3), 0, maxS);
      fwDrawChooser(); playTone(1200, 40); return;
    }
    if (tap && fwListState == 1 && x >= 58 && x <= 302 && y >= FW_ROW_Y0) {
      const int r = (y - FW_ROW_Y0) / FW_ROW_H, i = fwScroll + r;
      if (r < FW_ROWS && i < fwCount && (y - FW_ROW_Y0) % FW_ROW_H <= 38) {
        fwSel = i; fwMode = 2; fwInfoState = 0; fwSelSha = "";
        fwStartTask(2);
        fwDrawChooser(); playTone(1500, 60);
      }
    }
  } else if (fwMode == 2) {
    if (hSwipe || (tap && x >= 52 && x <= 178 && y >= 196 && y <= 272)) { fwMode = 1; fwDrawChooser(); playTone(1200, 50); return; }
    if (tap && fwInfoState == 1 && x >= 182 && x <= 308 && y >= 196 && y <= 272) {
      if (flightTimerRunning || automaticFlightRunning) return;
      if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) < OTA_MIN_FREE_HEAP) { snprintf(fwMsg, sizeof(fwMsg), "Low memory"); fwInfoState = 2; fwDrawChooser(); return; }
      playTone(1500, 80);
      fwMode = 3; fwDlPct = 0; fwDlDone = 0; fwDlLen = fwSizes[fwSel];
      otaSetStatus("FW: starting...", TFT_YELLOW);
      char path[64]; snprintf(path, sizeof(path), "%s/%s", FW_DIR, fwNames[fwSel]);
      otaDownloadPath(fwSelSha, fwSelEpoch, path, fwNames[fwSel]);   // se riesce lo strumento si riavvia
      fwDrawChooser();
    }
  } else if (fwMode == 3) {
    if (tap || hSwipe) { fwMode = 1; fwDrawChooser(); playTone(1200, 50); }
  }
}

void startLogWifi() {
  if (flightTimerRunning || automaticFlightRunning) return;
  fwMode = 0;
  bleSuspendForWifi();  // il BLE resta attivo; si ferma solo se la RAM interna e' troppo bassa
  otaAutoChecked = false;
  otaUpdatePending = false;
  ++otaSessionId;
  otaStatusDirty = false;
  otaStatusText[0] = '\0';
  if (!wifiLogRoutesReady) {
    wifiLogServer.on("/", HTTP_GET, wifiLogSendPage);
    wifiLogServer.on("/logs.csv", HTTP_GET, wifiLogSendCsv);
    wifiLogServer.on("/track.kml", HTTP_GET, wifiLogSendKml);
    wifiLogServer.on("/airspace", HTTP_GET, wifiAirspacePage);
    wifiLogServer.on("/airspace", HTTP_POST, wifiAirspaceFinish, wifiAirspaceUpload);
    wifiLogServer.on("/waypoints", HTTP_GET, wifiWaypointsPage);
    wifiLogServer.on("/waypoints", HTTP_POST, wifiWaypointsFinish, wifiWaypointsUpload);
    wifiLogServer.on("/update", HTTP_GET, wifiOtaShowPage);
    wifiLogServer.on("/update", HTTP_POST, wifiOtaFinishRequest, wifiOtaHandleUpload);
    wifiLogServer.onNotFound([]() { wifiLogServer.send(404, "text/plain", "Not found"); });
    wifiLogRoutesReady = true;
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_LOG_HOST);
  WiFi.begin(wifiLogSsid, wifiLogPass);
  wifiLogState = WIFI_LOG_CONNECTING;
  wifiLogStartedAt = millis();
}

void stopLogWifi() {
  if (wifiLogState == WIFI_LOG_OFF) return;
  if (wifiOtaUploadActive) {
    Update.abort();
    wifiOtaUploadActive = false;
    setWifiOtaError("Upload interrupted: Wi-Fi disattivato");
  }
  wifiLogServer.stop();
  MDNS.end();
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
  wifiLogState = WIFI_LOG_OFF;
  wifiLogDrawnState = WIFI_LOG_OFF;
  bleResumeAfterWifi();
}

void handleLogWifi() {
  if (wifiLogState == WIFI_LOG_OFF) return;
  // Sicurezza: niente WiFi in volo o fuori dalle schermate dei log.
  const bool inFlight = flightTimerRunning || automaticFlightRunning;
  const bool onWifiScreen = (currentScreen == WIFI_LOGS_SCREEN);
  if (inFlight || (!onWifiScreen && currentScreen != ICT_ALERT_SCREEN && currentScreen != FLIGHT_END_SCREEN &&
                        currentScreen != RISK_WARNING_SCREEN && currentScreen != SUNSET_SCREEN)) {
    stopLogWifi();
    if (onWifiScreen) {
      currentScreen = wifiReturnScreen;
      for (int i = 0; i < numScreens; ++i) {
        if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
      }
      redrawCurrentScreen();
    }
    return;
  }
  esp_task_wdt_reset();
  if (wifiLogState == WIFI_LOG_CONNECTING) {
    if (WiFi.status() == WL_CONNECTED) {
      wifiLogServer.begin();
      MDNS.begin(WIFI_LOG_HOST);
      MDNS.addService("http", "tcp", 80);
      wifiLogState = WIFI_LOG_CONNECTED;
    } else if (millis() - wifiLogStartedAt > WIFI_LOG_CONNECT_TIMEOUT_MS) {
      WiFi.disconnect(true, false);
      wifiLogState = WIFI_LOG_FAILED;
    }
  } else if (wifiLogState == WIFI_LOG_CONNECTED) {
    if (WiFi.status() != WL_CONNECTED) {
      wifiLogServer.stop();
      MDNS.end();
      WiFi.begin(wifiLogSsid, wifiLogPass);
      wifiLogState = WIFI_LOG_CONNECTING;
      wifiLogStartedAt = millis();
    } else if (!otaAutoChecked && onWifiScreen) {
      otaAutoChecked = true;
      otaStartCheckTask();
    } else {
      wifiLogServer.handleClient();
    }
  }
  if (otaStatusDirty) {
    otaStatusDirty = false;
    if (onWifiScreen) otaDrawStatus();
  }
  if (onWifiScreen && wifiLogDrawnState != wifiLogState) drawWifiLogsScreen();
  if (fwDirty) { fwDirty = false; if (onWifiScreen && fwMode != 0) fwDrawChooser(); }
}

void drawWifiLogsScreen() {
  wifiLogDrawnState = wifiLogState;
  if (fwMode != 0 && wifiLogState != WIFI_LOG_CONNECTED) fwMode = 0;   // WiFi perso: si torna alla pagina
  if (fwMode != 0) { fwDrawChooser(); return; }
  lcd.fillScreen(themeBg());
  lcd.setTextDatum(MC_DATUM);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
  lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
  setVividTextColor(TFT_CYAN);
  lcd.drawString("WIFI LOG", 180, 50);

  if (wifiLogState == WIFI_LOG_CONNECTED) {
    setVividTextColor(TFT_GREEN);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    // "Connected" centrata tra la base di "WIFI LOG" (baseline ~61) e l'apice di
    // "Open in phone" (~131): scritta alta ~19 px, quindi baseline a 106.
    lcd.setTextDatum(lgfx::textdatum_t::baseline_center);
    lcd.drawString("Connected", 180, 106);
    lcd.setTextDatum(MC_DATUM);
    // Font FreeSans Bold nativo piu grande: evita l'ingrandimento pixelato.
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold18pt7b);
    setVividTextColor(TFT_WHITE);
    // Testo su due righe per mantenerlo interamente entro i bordi del display.
    lcd.drawString("Open in phone", 180, 148);
    lcd.drawString("browser:", 180, 178);
    setVividTextColor(TFT_YELLOW);
    lcd.drawString(WiFi.localIP().toString().c_str(), 180, 222);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    setVividTextColor(TFT_CYAN);
    lcd.drawString("Browser: tap Update firmware", 180, 260);
    otaDrawStatus();
    fwDrawButton();   // CHOOSE FW: scelta del firmware per nome
    // lcd.drawString("Tap to close", 180, 310);
  } else if (wifiLogState == WIFI_LOG_FAILED) {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    setVividTextColor(TFT_RED);
    lcd.drawString("Hotspot not found", 180, 150);
    setVividTextColor(TFT_WHITE);
    lcd.drawString("Turn on phone hotspot", 180, 190);
    lcd.drawString("(2.4 GHz)", 180, 215);
    wifiDrawChangeLoginButton();
    setVividTextColor(TFT_CYAN);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    lcd.drawString("Swipe to exit", 180, 310);
  } else if (wifiLogState == WIFI_LOG_OFF) {
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    setVividTextColor(TFT_RED);
    lcd.drawString("Wi-Fi off during flight", 180, 170);
    setVividTextColor(TFT_CYAN);
    lcd.drawString("Swipe to exit", 180, 310);
  } else {
    // Stessa dimensione e stesso FreeSans Bold usati per "SUNSET".
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    lcd.setTextSize((1.0f) * THEME_TEXT_SCALE, (1.0f) * THEME_TEXT_SCALE);
    setVividTextColor(TFT_WHITE);
    lcd.drawString("Connecting to", 180, 112);
    // Istruzione per l'utente: a quale rete collegare il dispositivo
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    setVividTextColor(TFT_YELLOW);
    lcd.drawString("Router/Hotspot WIFI:", 180, 142);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold12pt7b);
    setVividTextColor(TFT_GREEN);
    lcd.drawString(wifiLogSsid, 180, 172);   // nome rete in verde
    const char *passwordLabel = "PASSWORD:";
    char passwordValue[70];
    snprintf(passwordValue, sizeof(passwordValue), " %s", wifiLogPass[0] ? wifiLogPass : "(none)");
    const int passwordLabelWidth = lcd.textWidth(passwordLabel);
    const int passwordLineWidth = passwordLabelWidth + lcd.textWidth(passwordValue);
    const int passwordLineX = (360 - passwordLineWidth) / 2;
    lcd.setTextDatum(ML_DATUM);
    setVividTextColor(TFT_WHITE);
    lcd.drawString(passwordLabel, passwordLineX, 205);
    setVividTextColor(TFT_GREEN);
    lcd.drawString(passwordValue, passwordLineX + passwordLabelWidth, 205);
    lcd.setTextDatum(MC_DATUM);
    wifiDrawChangeLoginButton();
    setVividTextColor(TFT_CYAN);
    lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
    lcd.drawString("Swipe to exit", 180, 310);
  }
}

// ========== WIFI LOGIN: nome rete e password modificabili dal dispositivo ==========
// Si apre dal pulsante CHANGE LOGIN della pagina WIFI LOG (mentre cerca l'hotspot o se non lo trova).
// Tastiera a schermo: tocca SSID o PASS per scegliere il campo; ABC = maiuscole, #+= = simboli,
// DEL cancella l'ultimo carattere. SAVE salva e riprova la connessione, CANCEL o uno swipe dx/sx esce senza salvare.
static const int WL_BTN_X = 95, WL_BTN_Y = 240, WL_BTN_W = 170, WL_BTN_H = 34;   // CHANGE LOGIN in WIFI LOG
static const int WL_KEY_X0 = 35, WL_KEY_W = 29, WL_KEY_Y0 = 114, WL_KEY_H = 34;  // tastiera 10 x 5 tasti
static const int WL_BTNS_Y = 290, WL_BTNS_H = 32;                                // CANCEL / SAVE
static const char *const WL_ROWS[3][4] = {
  {"1234567890", "qwertyuiop", "asdfghjkl-", "zxcvbnm._@"},
  {"1234567890", "QWERTYUIOP", "ASDFGHJKL-", "ZXCVBNM._@"},
  {"!#$%&*()+=", "/?:;,'\"<>~", "[]{}^|\\`-_", "@.,!?*+=$%"}
};
static char wifiLoginEditSsid[33];
static char wifiLoginEditPass[64];
static int wifiLoginField = 0;    // 0 = SSID, 1 = PASSWORD
static int wifiLoginKbMode = 0;   // 0 minuscole, 1 maiuscole, 2 simboli

void wifiDrawChangeLoginButton() {
  lcd.drawRoundRect(WL_BTN_X, WL_BTN_Y, WL_BTN_W, WL_BTN_H, 8, themeColor(TFT_ORANGE));
  lcd.drawRoundRect(WL_BTN_X + 1, WL_BTN_Y + 1, WL_BTN_W - 2, WL_BTN_H - 2, 7, themeColor(TFT_ORANGE));
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1);
  lcd.setTextDatum(MC_DATUM);
  setVividTextColor(TFT_ORANGE);
  lcd.drawString("CHANGE LOGIN", WL_BTN_X + WL_BTN_W / 2, WL_BTN_Y + WL_BTN_H / 2);
}
bool wifiTapOnChangeLogin(int tx, int ty) {
  if (wifiLogState != WIFI_LOG_CONNECTING && wifiLogState != WIFI_LOG_FAILED) return false;
  return tx >= WL_BTN_X - 12 && tx <= WL_BTN_X + WL_BTN_W + 12 &&
         ty >= WL_BTN_Y - 12 && ty <= WL_BTN_Y + WL_BTN_H + 12;
}

void loadWifiLogin() {
  if (!preferences.begin("wifi", true)) return;   // namespace mai scritto: restano i valori di serie
  const String ss = preferences.getString("ssid", WIFI_LOG_DEFAULT_SSID);
  const String pw = preferences.getString("pass", WIFI_LOG_DEFAULT_PASS);
  preferences.end();
  if (ss.length() > 0 && ss.length() <= 32) strlcpy(wifiLogSsid, ss.c_str(), sizeof(wifiLogSsid));
  if (pw.length() <= 63) strlcpy(wifiLogPass, pw.c_str(), sizeof(wifiLogPass));
}
static void saveWifiLogin() {
  if (preferences.begin("wifi", false)) {
    preferences.putString("ssid", wifiLogSsid);
    preferences.putString("pass", wifiLogPass);
    preferences.end();
  }
}

static void wifiLoginDrawTitle(const char *msg, uint16_t color) {
  lcd.fillRect(80, 10, 200, 32, themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1);
  lcd.setTextDatum(MC_DATUM);
  setVividTextColor(color);
  lcd.drawString(msg, 180, 27);
}

static void wifiLoginDrawField(int f) {
  const int x = (f == 0) ? 66 : 46, w = (f == 0) ? 228 : 268;
  const int y = (f == 0) ? 44 : 78, h = 28;
  const bool sel = (wifiLoginField == f);
  lcd.fillRect(x, y, w, h, themeBg());
  const uint16_t border = sel ? themeColor(TFT_YELLOW) : themeColor(TFT_DARKGREY);
  lcd.drawRoundRect(x, y, w, h, 6, border);
  if (sel) lcd.drawRoundRect(x + 1, y + 1, w - 2, h - 2, 5, border);
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1);
  lcd.setTextDatum(ML_DATUM);
  const char *label = (f == 0) ? "SSID" : "PASS";
  setVividTextColor(TFT_CYAN);
  lcd.drawString(label, x + 8, y + h / 2);
  const int textX = x + 8 + lcd.textWidth(label) + 8;
  const int avail = x + w - 12 - textX;
  const char *val = (f == 0) ? wifiLoginEditSsid : wifiLoginEditPass;
  const char *shown = val;
  while (*shown && lcd.textWidth(shown) > avail) ++shown;   // testo lungo: si vede la parte finale
  setVividTextColor(TFT_WHITE);
  if (*shown) lcd.drawString(shown, textX, y + h / 2);
  if (sel) {   // cursore
    const int cx = textX + (*shown ? lcd.textWidth(shown) : 0) + 2;
    lcd.fillRect(cx, y + 6, 2, h - 12, themeColor(TFT_YELLOW));
  }
}

static void wifiLoginDrawKey(int col, int row, int span, const char *label, uint16_t color) {
  const int x = WL_KEY_X0 + col * WL_KEY_W, y = WL_KEY_Y0 + row * WL_KEY_H, w = span * WL_KEY_W;
  lcd.drawRoundRect(x + 2, y + 2, w - 4, WL_KEY_H - 4, 5, themeColor(TFT_DARKGREY));
  setVividTextColor(color);
  lcd.drawString(label, x + w / 2, y + WL_KEY_H / 2);
}

static void wifiLoginDrawKeyboard() {
  lcd.fillRect(WL_KEY_X0, WL_KEY_Y0, 10 * WL_KEY_W, 5 * WL_KEY_H, themeBg());
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1);
  lcd.setTextDatum(MC_DATUM);
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 10; ++c) {
      const char key[2] = { WL_ROWS[wifiLoginKbMode][r][c], '\0' };
      wifiLoginDrawKey(c, r, 1, key, TFT_WHITE);
    }
  }
  wifiLoginDrawKey(0, 4, 2, wifiLoginKbMode == 1 ? "abc" : "ABC", wifiLoginKbMode == 1 ? TFT_YELLOW : TFT_CYAN);
  wifiLoginDrawKey(2, 4, 2, "#+=", wifiLoginKbMode == 2 ? TFT_YELLOW : TFT_CYAN);
  wifiLoginDrawKey(4, 4, 4, "SPACE", TFT_CYAN);
  wifiLoginDrawKey(8, 4, 2, "DEL", TFT_ORANGE);
}

void drawWifiLoginScreen() {
  lcd.fillScreen(themeBg());
  wifiLoginDrawTitle("WIFI LOGIN", TFT_CYAN);
  wifiLoginDrawField(0);
  wifiLoginDrawField(1);
  wifiLoginDrawKeyboard();
  lcd.drawRoundRect(74, WL_BTNS_Y, 102, WL_BTNS_H, 8, themeColor(TFT_ORANGE));
  lcd.drawRoundRect(75, WL_BTNS_Y + 1, 100, WL_BTNS_H - 2, 7, themeColor(TFT_ORANGE));
  lcd.drawRoundRect(184, WL_BTNS_Y, 102, WL_BTNS_H, 8, themeVivid(TFT_GREEN));
  lcd.drawRoundRect(185, WL_BTNS_Y + 1, 100, WL_BTNS_H - 2, 7, themeVivid(TFT_GREEN));
  lcd.setFreeFont(&lgfx::v1::fonts::FreeSansBold9pt7b);
  lcd.setTextSize(1);
  lcd.setTextDatum(MC_DATUM);
  setVividTextColor(TFT_ORANGE);
  lcd.drawString("CANCEL", 125, WL_BTNS_Y + WL_BTNS_H / 2);
  setVividTextColor(TFT_GREEN);
  lcd.drawString("SAVE", 235, WL_BTNS_Y + WL_BTNS_H / 2);
}

void openWifiLoginScreen() {
  stopLogWifi();   // niente WiFi mentre si scrive: riparte con i nuovi dati dopo SAVE (o i vecchi con CANCEL)
  strlcpy(wifiLoginEditSsid, wifiLogSsid, sizeof(wifiLoginEditSsid));
  strlcpy(wifiLoginEditPass, wifiLogPass, sizeof(wifiLoginEditPass));
  wifiLoginField = 0;
  wifiLoginKbMode = 0;
  currentScreen = WIFI_LOGIN_SCREEN;
  drawWifiLoginScreen();
}

static void wifiLoginExit(bool save) {
  if (save) {
    strlcpy(wifiLogSsid, wifiLoginEditSsid, sizeof(wifiLogSsid));
    strlcpy(wifiLogPass, wifiLoginEditPass, sizeof(wifiLogPass));
    saveWifiLogin();
    AVEA_LOGF("WIFI LOGIN: salvata rete \"%s\"\n", wifiLogSsid);
  }
  currentScreen = WIFI_LOGS_SCREEN;
  for (int i = 0; i < numScreens; ++i) {
    if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
  }
  startLogWifi();
  drawWifiLogsScreen();
}

void wifiLoginTouchRelease(bool tapValid, int diffX, int diffY, int tx, int ty) {
  if (!tapValid) {
    if (abs(diffX) > SWIPE_MIN_X_PX && abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX) {
      playTone(1500, 80);
      wifiLoginExit(false);   // swipe dx/sx: esce senza salvare
    }
    return;
  }
  if (ty >= 40 && ty < 76) {          // campo SSID
    wifiLoginField = 0;
    wifiLoginDrawField(0); wifiLoginDrawField(1);
    playTone(1200, 30);
    return;
  }
  if (ty >= 76 && ty < WL_KEY_Y0) {   // campo PASSWORD
    wifiLoginField = 1;
    wifiLoginDrawField(0); wifiLoginDrawField(1);
    playTone(1200, 30);
    return;
  }
  if (ty >= WL_KEY_Y0 && ty < WL_KEY_Y0 + 5 * WL_KEY_H && tx >= WL_KEY_X0 && tx < WL_KEY_X0 + 10 * WL_KEY_W) {
    const int r = (ty - WL_KEY_Y0) / WL_KEY_H;
    const int c = (tx - WL_KEY_X0) / WL_KEY_W;
    char *buf = (wifiLoginField == 0) ? wifiLoginEditSsid : wifiLoginEditPass;
    const size_t cap = (wifiLoginField == 0) ? 32 : 63;
    const size_t len = strlen(buf);
    char add = 0;
    if (r < 4) {
      add = WL_ROWS[wifiLoginKbMode][r][c];
    } else if (c <= 1) {
      wifiLoginKbMode = (wifiLoginKbMode == 1) ? 0 : 1;   // ABC / abc
      wifiLoginDrawKeyboard();
    } else if (c <= 3) {
      wifiLoginKbMode = (wifiLoginKbMode == 2) ? 0 : 2;   // simboli
      wifiLoginDrawKeyboard();
    } else if (c <= 7) {
      add = ' ';
    } else if (len > 0) {
      buf[len - 1] = '\0';                                 // DEL
    }
    if (add != 0) {
      if (len >= cap) { playTone(600, 80); return; }       // campo pieno
      buf[len] = add;
      buf[len + 1] = '\0';
    }
    wifiLoginDrawTitle("WIFI LOGIN", TFT_CYAN);
    wifiLoginDrawField(wifiLoginField);
    playTone(1200, 20);
    return;
  }
  if (ty >= WL_BTNS_Y - 6 && ty <= WL_BTNS_Y + WL_BTNS_H + 10) {
    if (tx >= 66 && tx < 180) {          // CANCEL
      playTone(1500, 80);
      wifiLoginExit(false);
    } else if (tx >= 180 && tx <= 294) { // SAVE
      const size_t pl = strlen(wifiLoginEditPass);
      if (wifiLoginEditSsid[0] == '\0') {
        wifiLoginDrawTitle("SSID missing", TFT_RED);
        playTone(600, 150);
      } else if (pl > 0 && pl < 8) {
        wifiLoginDrawTitle("Password min 8 chars", TFT_RED);
        playTone(600, 150);
      } else {
        playTone(1500, 80);
        wifiLoginExit(true);
      }
    }
  }
}

// ========== SETUP ==========
// ========== BLUETOOTH LE: MASTER -> SLAVE ==========
// Pacchetti BLE compatti (<= 20 byte, nessuna dipendenza dall'MTU negoziato):
//  tipo 1 GPS, 2 sensori, 3 timer automatico, 4 frammento log, 5 orari, 6 stato ICT/schermate.
#define BLE_SVC_UUID "7a1c0001-5e3d-4b8a-9d4e-a0e1a7e00001"
#define BLE_CHR_UUID "7a1c0002-5e3d-4b8a-9d4e-a0e1a7e00001"
#define BLE_CMD_UUID "7a1c0003-5e3d-4b8a-9d4e-a0e1a7e00001"   // slave -> master: comandi (scrittura)
#define BLE_MET_UUID "7a1c0004-5e3d-4b8a-9d4e-a0e1a7e00001"   // telefono -> strumento: QNH e vento (app AVEA Link)
#define BLE_DST_UUID "7a1c0005-5e3d-4b8a-9d4e-a0e1a7e00001"   // strumento -> telefono: coordinate della destinazione
#define BLE_MIN_FREE_INTERNAL 80000UL   // RAM interna minima per avviare lo stack BLE
#define BLE_DATA_TIMEOUT_MS 1500UL      // oltre questo tempo i dati ricevuti sono considerati scaduti
// Quanti apparecchi possono stare attaccati al master insieme. Il conto comprende il TELEFONO con
// AVEA Link, che e' un client come gli altri perche' usa lo stesso servizio: due slave piu' il
// telefono fanno tre. Con il vecchio limite di due, telefono piu' un solo slave bastavano a far
// smettere l'annuncio e il secondo slave non trovava piu' il master. La libreria con cui questo
// firmware viene compilato regge quattro collegamenti insieme e sei attivita' nel controllore,
// quindi tre stanno larghi; ognuno costa un po' di RAM interna, percio' non si esagera.
#define BLE_MAX_CLIENTS 3

#pragma pack(push, 1)
struct BlePktGps {
  uint8_t type;      // 1
  uint8_t flags;     // bit0 = fix GPS valido
                     // bit1 = GPS, bit2 = GLONASS, bit3 = Galileo, bit4 = BDS, bit5 = QZSS
  float lat, lon;
  uint16_t speed10;  // km/h * 10
  uint16_t course10; // gradi * 10
  int16_t altM;      // quota GPS in m
  uint8_t sats;
  uint8_t h, m, s;   // UTC
};
struct BlePktSens {
  uint8_t type;      // 2
  uint8_t flags;     // bit0 quota valida, bit1 vario valido, bit2 meteo valido, bit3 (G-meter, non piu' usato), bit4 sonda OAT
  int16_t vario100;  // m/s * 100
  int32_t altCm;     // quota filtrata in cm
  int16_t temp10;    // gradi C * 10
  uint8_t hum2;      // % * 2
  int16_t g100;      // sonda esterna (OAT) in gradi C * 10 se flags bit4 (era il G-meter, rimosso)
  uint16_t qnh10;    // hPa * 10
  uint8_t d, mo, y;  // data UTC (anno - 2000)
};
struct BlePktTimer {
  uint8_t type;
  uint8_t running;
  uint32_t elapsedMs;
};
struct BlePktTimes { uint8_t type; char start[9]; char stop[9]; };
struct BlePktStatus {
  uint8_t type; uint8_t flags; uint8_t ictMinutes; uint32_t ictElapsedMs;
  uint8_t flightEndPrevious; uint8_t ictPrevious; uint8_t tzIndex;
};
struct BlePktLog { uint8_t type; uint8_t index; uint8_t data[18]; };
// Comando slave -> master (tipo 7). cmd: 1 chiudi fine volo, 2 imposta QNH (value = hPa*10), 3 demo ON, 4 demo OFF, 5 fuso UTC (value = indice)
struct BlePktCmd { uint8_t type; uint8_t cmd; uint16_t value; };
// Home (tipo 8): master -> slave in notifica, slave -> master sulla caratteristica comandi.
// Gradi * 1e7 in int32: master e slave salvano esattamente le stesse cifre.
struct BlePktHome { uint8_t type; uint8_t valid; int32_t lat7; int32_t lon7; };
// Carburante (tipo 9): master -> slave in notifica (1 Hz), slave -> master sulla caratteristica comandi.
struct BlePktFuel { uint8_t type; uint8_t tank; uint8_t lph2; uint8_t resMin; uint8_t gnd2; uint8_t full; uint16_t liters10; };
// Soglie dell'arco velocita' (tipo 10): master -> slave in notifica (1 Hz), in km/h su 16 bit (fino a 500 km/h).
// Slave -> master: BlePktCmd cmd 8, value = giallo (9 bit, 0-511) | (rosso - giallo) << 9 (7 bit, 0-127).
// Master e slave devono avere lo stesso firmware (prima le soglie erano su 8 bit).
struct BlePktArc { uint8_t type; uint16_t yel; uint16_t red; };
// Meteo del telefono (tipo 13): master -> slave in notifica, 1 Hz. Il telefono puo' collegarsi solo
// al master, perche' il servizio Bluetooth esiste solo in quel ruolo: senza questo pacchetto lo slave
// non avrebbe ne' la pagina ONLINE WEATHER ne' la freccia del vento. L'eta' viaggia in minuti, cosi'
// sullo slave la riga "n min" dice l'eta' vera del dato e non da quando l'ha ricevuto lui.
struct BlePktMet {
  uint8_t type;        // 13
  uint8_t flags;       // bit0 QNH, bit1 vento, bit2 visibilita', bit3 pioggia, bit4 congelamento,
                       // bit5 rugiada, bit6 meteo del campo di adesso (metAligned del master)
  uint8_t ageMin;      // eta' del dato in minuti
  uint8_t dir2;        // direzione del vento in gradi / 2
  uint16_t qnh10;      // hPa * 10
  uint16_t kt10;       // nodi * 10
  uint16_t gust10;     // nodi * 10
  uint16_t visM;       // metri
  uint8_t precip10;    // mm * 10 (fino a 25,5)
  uint8_t prob;        // probabilita' di pioggia %
  uint16_t frzM;       // quota di congelamento in metri
  int16_t dew10;       // punto di rugiada * 10
};
// Sbandometro (tipo 14): master -> slave in notifica, 10 Hz. L'accelerometro sta solo sul master,
// quindi la pallina che lo slave disegna e' quella del master, con lo zero del master. Pacchetto a
// parte e non dentro al tipo 2, cosi' uno slave con firmware vecchio lo ignora senza perdere il resto.
struct BlePktSlip {
  uint8_t type;        // 14
  uint8_t flags;       // bit0 = rateo di virata attendibile (giroscopio tarato)
  int16_t g1000;       // forza laterale in g * 1000
  int16_t rate10;      // rateo di virata in gradi/s * 10
};
// BlePktPrefs (tipo 11) e' definita in cima al file: serve ai prototipi generati da Arduino.
#pragma pack(pop)
static_assert(sizeof(BlePktGps) == 20, "BlePktGps deve stare in 20 byte");
static_assert(sizeof(BlePktSens) <= 20, "BlePktSens deve stare in 20 byte");
static_assert(sizeof(BlePktTimer) <= 20, "BlePktTimer deve stare in 20 byte");
static_assert(sizeof(BlePktTimes) <= 20, "BlePktTimes deve stare in 20 byte");
static_assert(sizeof(BlePktStatus) <= 20, "BlePktStatus deve stare in 20 byte");
static_assert(sizeof(BlePktLog) <= 20, "BlePktLog deve stare in 20 byte");
static_assert(sizeof(BlePktHome) <= 20, "BlePktHome deve stare in 20 byte");
static_assert(sizeof(BlePktFuel) <= 20, "BlePktFuel deve stare in 20 byte");
static_assert(sizeof(BlePktArc) <= 20, "BlePktArc deve stare in 20 byte");
static_assert(sizeof(BlePktPrefs) <= 20, "BlePktPrefs deve stare in 20 byte");
static_assert(sizeof(BlePktMet) <= 20, "BlePktMet deve stare in 20 byte");
static_assert(sizeof(BlePktSlip) <= 20, "BlePktSlip deve stare in 20 byte");
static_assert((sizeof(FlightLog) + 17) / 18 <= 16, "FlightLog: troppi frammenti BLE");

struct BleRemote {
  bool gpsValid;
  bool hasGps, hasGlonass, hasGalileo, hasBds, hasQzss;
  float lat, lon, speedKmh, course, gpsAlt;
  int sats, h, m, s;
  unsigned long gpsAtMs;
  uint8_t sensFlags;
  float vario, alt, tempC, hum, qnh;
  float oatC;        // sonda esterna del master (flags bit4)
  int day, month, year;
  unsigned long sensAtMs;
  bool automaticRunning;
  uint32_t automaticElapsedMs;
  unsigned long timerAtMs;
  uint8_t statusFlags, remoteIctMinutes, remoteFlightEndPrevious, remoteIctPrevious, remoteTzIndex;
  uint32_t remoteIctElapsedMs;
  unsigned long statusAtMs;
  uint8_t remoteLogBytes[sizeof(FlightLog)];
  bool homeValid;
  int32_t homeLat7, homeLon7;
  unsigned long homeAtMs;
  BlePktFuel fuel;
  unsigned long fuelAtMs;
  uint16_t arcYel, arcRed;      // soglie arco velocita' del master (km/h)
  unsigned long arcAtMs;
  BlePktPrefs prefs;            // impostazioni condivise del master
  unsigned long prefsAtMs;
  BlePktMet met;                // meteo del telefono, girato dal master
  unsigned long metAtMs;
  BlePktSlip slip;              // sbandometro del master
  unsigned long slipAtMs;
  uint16_t remoteLogMask;
  bool remoteLogReady;
};
static BleRemote bleRemote = {};
static portMUX_TYPE bleMux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool bleLinkUp = false;
static volatile bool bleNeedAdv = false;
static volatile bool bleFound = false;
static volatile uint8_t bleConnCount = 0;
static bool bleStackUp = false;
static BLEServer *bleServer = nullptr;
static BLECharacteristic *bleChar = nullptr;
static BLEClient *bleCli = nullptr;
static BLECharacteristic *bleCmdChar = nullptr;          // master: caratteristica su cui lo slave scrive i comandi
static BLERemoteCharacteristic *bleCmdRemote = nullptr;  // slave: caratteristica comandi del master
// Comandi ricevuti dal master (impostati nella callback BLE, eseguiti dal loop())
static volatile uint8_t bleCmdRxBits = 0;     // bit0 chiudi fine volo, bit1 QNH, bit2 demo
static volatile uint16_t bleCmdRxQnh10 = 0;
static volatile uint16_t bleCmdRxIct = 0;
static volatile bool bleCmdRxDemoOn = false;
static volatile uint8_t bleCmdRxTz = 0;
static volatile bool bleCmdRxHomeValid = false;   // home ricevuta dallo slave (bit 0x40)
static volatile int32_t bleCmdRxHomeLat7 = 0, bleCmdRxHomeLon7 = 0;
static volatile bool bleCmdTxHomeValid = false;   // slave: home da inviare al master (bit 0x40)
static volatile int32_t bleCmdTxHomeLat7 = 0, bleCmdTxHomeLon7 = 0;
static volatile unsigned long bleHomeLocalAt = 0;  // slave: ultima home cambiata localmente
static BlePktFuel bleCmdRxFuel = {};                 // master: carburante ricevuto dallo slave (bit 0x80)
static BlePktFuel bleCmdTxFuel = {};                 // slave: carburante da inviare al master (bit 0x80)
static volatile unsigned long bleFuelLocalAt = 0;  // slave: ultimo SAVE di FUEL SETUP fatto qui
// Comandi in coda sullo slave (inviati dal task BLE)
static volatile uint8_t bleCmdTxBits = 0;
static volatile uint16_t bleCmdTxQnh10 = 0;
static volatile bool bleCmdTxDemoOn = false;
static volatile uint8_t bleCmdTxTz = 0;
static volatile uint16_t bleCmdTxIct = 0;
static volatile unsigned long bleIctLocalAt = 0; // evita che lo stato precedente del master annulli una modifica locale
static volatile unsigned long bleTzLocalAt = 0;   // slave: ultimo fuso UTC cambiato localmente
static volatile unsigned long bleQnhLocalAt = 0;   // slave: ultimo QNH cambiato localmente (evita che il valore di ritorno del master lo sovrascriva)
// Soglie arco velocita' (bleCmdRxBits / bleCmdTxBits sono pieni: flag a parte)
static volatile bool bleCmdRxArc = false;           // master: soglie ricevute dallo slave
static volatile uint16_t bleCmdRxArcVal = 0;
static volatile bool bleCmdTxArc = false;           // slave: soglie da inviare al master
static volatile uint16_t bleCmdTxArcVal = 0;
static volatile unsigned long bleArcLocalAt = 0;    // slave: ultima modifica locale delle soglie
// Azzeramento dello sbandometro chiesto dallo slave: l'accelerometro e' del master, quindi il
// doppio tocco sull'ampolla dello slave deve azzerare il master (flag a parte, i bit sono pieni)
static volatile bool bleCmdRxSlipZero = false;      // master: azzeramento chiesto dallo slave
static volatile bool bleCmdTxSlipZero = false;      // slave: azzeramento da chiedere al master
static BlePktPrefs bleCmdRxPrefs = {};              // master: impostazioni ricevute dallo slave
static volatile bool bleCmdRxPrefsNew = false;
static BlePktPrefs bleCmdTxPrefs = {};              // slave: impostazioni da inviare al master
static volatile bool bleCmdTxPrefsNew = false;
static volatile unsigned long blePrefsLocalAt = 0;  // slave: ultima modifica locale delle impostazioni condivise
static BLEAdvertisedDevice *bleFoundDev = nullptr;
// ---- Avvisi "letti" su master e slave insieme: un tocco su uno chiude lo stesso avviso anche sull'altro ----
// Tipo di avviso: 1 = DENSITY ALTITUDE, 2 = CARB ICE, 3 = VISIBILITY. Si chiude solo quello mostrato:
// l'avviso successivo va toccato di nuovo (su uno qualsiasi dei due).
static volatile uint8_t bleAckTxKind = 0;            // slave: avviso toccato qui, da mandare al master
static volatile unsigned long bleAckTxAt = 0;
static volatile uint8_t bleAckRxKind = 0;            // master: avviso toccato sullo slave (cmd 9)
static volatile uint8_t bleAckSeq = 0, bleAckKind = 0;   // master: ultimo avviso letto, inviato agli slave (tipo 12)
static volatile unsigned long bleAckSendUntil = 0;
static volatile uint8_t bleAckRemoteSeq = 0, bleAckRemoteKind = 0;   // slave: ultimo tipo 12 ricevuto
static volatile bool bleAckRemoteNew = false;

static void bleNotify() {
  if (bleChar != nullptr) bleChar->notify();
}

class BleServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    if (bleConnCount < 255) bleConnCount++;
    bleLinkUp = true;
    if (bleConnCount < BLE_MAX_CLIENTS) bleNeedAdv = true;  // resta visibile per chi manca ancora
  }
  void onDisconnect(BLEServer *) override {
    if (bleConnCount > 0) bleConnCount--;
    bleLinkUp = (bleConnCount > 0);
    bleNeedAdv = true;
  }
};
static BleServerCb bleServerCb;

// Master: riceve i comandi dello slave. Qui si impostano solo flag; l'azione (display, NVS) la fa il loop().
class BleCmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *d = c->getData();
    const size_t n = c->getLength();
    if (d != nullptr && n >= sizeof(BlePktFuel) && d[0] == 9) {
      portENTER_CRITICAL(&bleMux);
      memcpy(&bleCmdRxFuel, d, sizeof(BlePktFuel));
      bleCmdRxBits |= 0x80;
      portEXIT_CRITICAL(&bleMux);
      return;
    }
    if (d != nullptr && n >= sizeof(BlePktPrefs) && d[0] == 11) {
      portENTER_CRITICAL(&bleMux);
      memcpy(&bleCmdRxPrefs, d, sizeof(BlePktPrefs));
      bleCmdRxPrefsNew = true;
      portEXIT_CRITICAL(&bleMux);
      return;
    }
    if (d != nullptr && n >= sizeof(BlePktHome) && d[0] == 8) {
      BlePktHome h;
      memcpy(&h, d, sizeof(h));
      portENTER_CRITICAL(&bleMux);
      bleCmdRxHomeValid = h.valid != 0; bleCmdRxHomeLat7 = h.lat7; bleCmdRxHomeLon7 = h.lon7;
      bleCmdRxBits |= 0x40;
      portEXIT_CRITICAL(&bleMux);
      return;
    }
    if (d == nullptr || n < sizeof(BlePktCmd) || d[0] != 7) return;
    BlePktCmd p;
    memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    if (p.cmd == 1) bleCmdRxBits |= 0x01;
    else if (p.cmd == 2) { bleCmdRxQnh10 = p.value; bleCmdRxBits |= 0x02; }
    else if (p.cmd == 3 || p.cmd == 4) { bleCmdRxDemoOn = (p.cmd == 3); bleCmdRxBits |= 0x04; }
    else if (p.cmd == 5) { bleCmdRxTz = (uint8_t)p.value; bleCmdRxBits |= 0x08; }
    else if (p.cmd == 6) { bleCmdRxIct = p.value; bleCmdRxBits |= 0x10; } // ICT: bit7 ON, bit0..4 minuti
    else if (p.cmd == 7) { bleCmdRxBits |= 0x20; } // chiusura allarme ICT
    else if (p.cmd == 8) { bleCmdRxArcVal = p.value; bleCmdRxArc = true; } // soglie arco velocita'
    else if (p.cmd == 9) { bleAckRxKind = (uint8_t)p.value; } // avviso letto sullo slave
    else if (p.cmd == 10) { bleCmdRxSlipZero = true; }        // azzeramento sbandometro dallo slave
    portEXIT_CRITICAL(&bleMux);
  }
};
static BleCmdCb bleCmdCb;

// ---- Meteo dal telefono (app AVEA Link) -------------------------------------------------
// Dieci byte: versione, flag, QNH x10, direzione del vento, nodi x10, raffica x10.
// Interi a 16 bit con il byte basso per primo. L'ora di arrivo la mette lo strumento.
class BleMetCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    const uint8_t *d = c->getData();
    const size_t n = c->getLength();
    if (d == nullptr || n < 10) return;
    const uint8_t ver = d[0];
    if (ver != 1 && ver != 2 && ver != 3) return;
    const uint16_t q   = (uint16_t)(d[2] | ((uint16_t)d[3] << 8));
    const uint16_t dir = (uint16_t)(d[4] | ((uint16_t)d[5] << 8));
    const uint16_t kt  = (uint16_t)(d[6] | ((uint16_t)d[7] << 8));
    const uint16_t gu  = (uint16_t)(d[8] | ((uint16_t)d[9] << 8));
    if (d[1] & 0x01) {
      metQnh = q / 10.0f;
      metQnhValid = (q >= 8500 && q <= 11000);   // 850..1100 hPa: fuori di li' e' spazzatura
    }
    if (d[1] & 0x02) {
      metWindDir = (int)(dir % 360);
      metWindKt = kt / 10.0f;
      metGustKt = gu / 10.0f;
      metWindValid = (kt <= 2000);               // oltre 200 nodi non e' vento, e' un errore
    }
    // Dalla versione 2 in poi: altri dieci byte con visibilita', pioggia, congelamento e rugiada.
    // Anche la versione 3 li porta, negli stessi byte: va letta anche lei, non solo la 2.
    if (ver >= 2 && n >= 20) {
      const uint16_t vis = (uint16_t)(d[10] | ((uint16_t)d[11] << 8));
      const uint16_t pre = (uint16_t)(d[12] | ((uint16_t)d[13] << 8));
      const uint16_t frz = (uint16_t)(d[15] | ((uint16_t)d[16] << 8));
      const int16_t dew = (int16_t)(d[17] | ((uint16_t)d[18] << 8));
      if (d[1] & 0x04) { metVisM = (int)vis; metVisValid = true; }
      if (d[1] & 0x08) { metPrecipMm = pre / 100.0f; metPrecipProb = (int)d[14]; metPrecipValid = true; }
      if (d[1] & 0x10) { metFrzM = (int)frz; metFrzValid = true; }
      if (d[1] & 0x20) { metDewC = dew / 10.0f; metDewValid = true; }
    }
    if (ver == 3 && n >= 28) {
      int32_t la = 0, lo = 0;
      memcpy(&la, d + 20, 4);
      memcpy(&lo, d + 24, 4);
      if ((d[1] & 0x40) && (la != 0 || lo != 0)) {
        metPhoneLat = la / 1e7;
        metPhoneLon = lo / 1e7;
        metPhonePosValid = true;
      }
    }
    // Primo pacchetto dopo un cambio di campo: da qui i dati sono di questo campo. Si
    // accende il pulsare del pallino e si segna che la pagina meteo puo' mostrarsi da sola.
    const bool eraAllineato = metAligned();
    metAtMs = millis();
    metExtraAgeMs = 0;   // dal telefono: appena misurato, nessuna eta' in piu'
    if (!eraAllineato) {
      metAlignedAtMs = metAtMs;
      metWxAutoPending = true;
    }
    AVEA_LOGF("[MET] dal telefono: QNH %.1f, vento %03d/%.0f kt, raffica %.0f, vis %d m, frz %d m\n",
              (double)metQnh, (int)metWindDir, (double)metWindKt, (double)metGustKt,
              (int)metVisM, (int)metFrzM);
  }
};
static BleMetCb bleMetCb;
static BLECharacteristic *bleMetChar = nullptr;
static BLECharacteristic *bleDstChar = nullptr;

// Destinazione (GO TO o casa) messa a disposizione del telefono: cosi' l'app puo' chiedere il
// meteo del campo dove stai andando invece che di dove ti trovi. Nove byte: un flag e due
// interi con segno, latitudine e longitudine moltiplicate per 10 milioni, byte basso per primo.
void bleUpdateDestination() {
  const bool ok = navTargetValid();
  // Cambio di campo: la destinazione compare, sparisce, oppure si sposta di oltre un
  // chilometro. Da questo istante il meteo in memoria non e' piu' di questo campo. Lo
  // strumento lo sa perche' la destinazione l'ha cambiata lui, senza chiedere niente al
  // telefono. Il conto della distanza e' piatto: a questi scopi basta e avanza.
  {
    static bool lastOk = false;
    static double lastLa = 0.0, lastLo = 0.0;
    const double la0 = ok ? navTargetLat() : 0.0;
    const double lo0 = ok ? navTargetLon() : 0.0;
    bool cambiata = (ok != lastOk);
    if (!cambiata && ok) {
      const double dLat = (la0 - lastLa) * 111.2;
      const double dLon = (lo0 - lastLo) * 111.2 * cos(lastLa * DEG_TO_RAD);
      cambiata = (sqrt(dLat * dLat + dLon * dLon) > 1.0);
    }
    lastOk = ok; lastLa = la0; lastLo = lo0;
    if (cambiata) {
      metDestChangedMs = millis();
      if (metDestChangedMs == 0) metDestChangedMs = 1;   // 0 vuol dire "mai cambiata"
    }
  }
  if (bleDstChar == nullptr) return;
  uint8_t b[9];
  b[0] = ok ? 1 : 0;
  const int32_t la = ok ? (int32_t)llround(navTargetLat() * 1e7) : 0;
  const int32_t lo = ok ? (int32_t)llround(navTargetLon() * 1e7) : 0;
  memcpy(b + 1, &la, 4);
  memcpy(b + 5, &lo, 4);
  bleDstChar->setValue(b, sizeof(b));
}

class BleClientCb : public BLEClientCallbacks {
  void onConnect(BLEClient *) override {}
  void onDisconnect(BLEClient *) override { bleLinkUp = false; }
};
static BleClientCb bleClientCb;

class BleScanCb : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice dev) override {
    if (dev.haveServiceUUID() && dev.isAdvertisingService(BLEUUID(BLE_SVC_UUID))) {
      if (bleFoundDev) { delete bleFoundDev; bleFoundDev = nullptr; }
      bleFoundDev = new BLEAdvertisedDevice(dev);
      bleFound = true;
      BLEDevice::getScan()->stop();
    }
  }
};
static BleScanCb bleScanCb;

static void bleHandlePacket(const uint8_t *d, size_t len) {
  if (d == nullptr || len < 2) return;
  const unsigned long now = millis();
  if (d[0] == 1 && len >= sizeof(BlePktGps)) {
    BlePktGps p;
    memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    bleRemote.gpsValid   = (p.flags & 0x01) != 0;
    bleRemote.hasGps     = (p.flags & 0x02) != 0;
    bleRemote.hasGlonass = (p.flags & 0x04) != 0;
    bleRemote.hasGalileo = (p.flags & 0x08) != 0;
    bleRemote.hasBds     = (p.flags & 0x10) != 0;
    bleRemote.hasQzss    = (p.flags & 0x20) != 0;
    bleRemote.lat = p.lat;
    bleRemote.lon = p.lon;
    bleRemote.speedKmh = p.speed10 / 10.0f;
    bleRemote.course = p.course10 / 10.0f;
    bleRemote.gpsAlt = (float)p.altM;
    bleRemote.sats = p.sats;
    bleRemote.h = p.h; bleRemote.m = p.m; bleRemote.s = p.s;
    bleRemote.gpsAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 3 && len >= sizeof(BlePktTimer)) {
    BlePktTimer p;
    memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    bleRemote.automaticRunning = p.running != 0;
    bleRemote.automaticElapsedMs = p.elapsedMs;
    bleRemote.timerAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 5 && len >= sizeof(BlePktTimes)) {
    BlePktTimes p; memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    memcpy(automaticStartUtc, p.start, 9); automaticStartUtc[15] = '\0';
    memcpy(automaticStopUtc, p.stop, 9); automaticStopUtc[15] = '\0';
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 6 && len >= sizeof(BlePktStatus)) {
    BlePktStatus p; memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    bleRemote.statusFlags = p.flags; bleRemote.remoteIctMinutes = p.ictMinutes;
    bleRemote.remoteIctElapsedMs = p.ictElapsedMs;
    bleRemote.remoteFlightEndPrevious = p.flightEndPrevious;
    bleRemote.remoteIctPrevious = p.ictPrevious; bleRemote.remoteTzIndex = p.tzIndex; bleRemote.statusAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 9 && len >= sizeof(BlePktFuel)) {
    portENTER_CRITICAL(&bleMux);
    memcpy(&bleRemote.fuel, d, sizeof(BlePktFuel));
    bleRemote.fuelAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 11 && len >= sizeof(BlePktPrefs)) {
    portENTER_CRITICAL(&bleMux);
    memcpy(&bleRemote.prefs, d, sizeof(BlePktPrefs));
    bleRemote.prefsAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 12 && len >= 3) {   // avviso letto (master -> slave): tipo, sequenza, avviso
    portENTER_CRITICAL(&bleMux);
    if (d[1] != bleAckRemoteSeq) { bleAckRemoteSeq = d[1]; bleAckRemoteKind = d[2]; bleAckRemoteNew = true; }
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 13 && len >= sizeof(BlePktMet)) {
    portENTER_CRITICAL(&bleMux);
    memcpy(&bleRemote.met, d, sizeof(BlePktMet));
    bleRemote.metAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 14 && len >= sizeof(BlePktSlip)) {
    portENTER_CRITICAL(&bleMux);
    memcpy(&bleRemote.slip, d, sizeof(BlePktSlip));
    bleRemote.slipAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 10 && len >= sizeof(BlePktArc)) {
    BlePktArc p; memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    bleRemote.arcYel = p.yel; bleRemote.arcRed = p.red; bleRemote.arcAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 8 && len >= sizeof(BlePktHome)) {
    BlePktHome p; memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    bleRemote.homeValid = p.valid != 0; bleRemote.homeLat7 = p.lat7; bleRemote.homeLon7 = p.lon7;
    bleRemote.homeAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  } else if (d[0] == 4 && len >= 2) {
    const uint8_t idx = d[1];
    const size_t count = (sizeof(FlightLog) + 17) / 18;
    if (idx < count && len >= 2 + min((size_t)18, sizeof(FlightLog) - (size_t)idx * 18)) {
      portENTER_CRITICAL(&bleMux);
      memcpy(&bleRemote.remoteLogBytes[(size_t)idx * 18], d + 2,
             min((size_t)18, sizeof(FlightLog) - (size_t)idx * 18));
      bleRemote.remoteLogMask |= (uint16_t)(1U << idx);
      const uint16_t needed = (uint16_t)((1U << count) - 1U);
      if ((bleRemote.remoteLogMask & needed) == needed) bleRemote.remoteLogReady = true;
      portEXIT_CRITICAL(&bleMux);
    }
  } else if (d[0] == 2 && len >= sizeof(BlePktSens)) {
    BlePktSens p;
    memcpy(&p, d, sizeof(p));
    portENTER_CRITICAL(&bleMux);
    bleRemote.sensFlags = p.flags;
    bleRemote.vario = p.vario100 / 100.0f;
    bleRemote.alt = p.altCm / 100.0f;
    bleRemote.tempC = p.temp10 / 10.0f;
    bleRemote.hum = p.hum2 / 2.0f;
    bleRemote.oatC = p.g100 / 10.0f;
    bleRemote.qnh = p.qnh10 / 10.0f;
    bleRemote.day = p.d; bleRemote.month = p.mo;
    bleRemote.year = p.y > 0 ? 2000 + p.y : 0;
    bleRemote.sensAtMs = now;
    portEXIT_CRITICAL(&bleMux);
  }
}

static void bleNotifyCb(BLERemoteCharacteristic *, uint8_t *data, size_t len, bool) {
  bleHandlePacket(data, len);
}

// ---- Impostazioni condivise master <-> slave (tipo 11) ----
// Quota m/ft (SET QNH), km/h <-> nodi (ALTIMETER e GPS SPEED), QFE m/ft, RESET QFE, fascia di circuito
// del QFE, larghezza e tolleranza del corridoio (LUBBER LINE). Tema, pagine e formato coordinate restano
// di ogni strumento.
static BlePktPrefs blePrefsFromLocal() {
  BlePktPrefs p;
  p.type = 11;
  p.flags = (altitudeUnitsIsMeters ? 0x01 : 0) | (gpsSpeedShowKnots ? 0x02 : 0) | (qfeShowFeet ? 0x04 : 0) |
            (qfeReferenceValid ? 0x08 : 0);
  p.lubW = (uint8_t)lubberWidthIdx;
  p.lubY = (uint8_t)lubberYellowIdx;
  p.qfeRefCm = qfeReferenceValid ? (int32_t)lroundf(qfeReferenceAltitudeMeters * 100.0f) : 0;
  p.qfeLow = (uint16_t)constrain(qfeCircuitLowM, 0, 9000);
  p.qfeHigh = (uint16_t)constrain(qfeCircuitHighM, 0, 9000);
  p.gotoIdx = navGotoActive ? (uint16_t)navGotoIdx : 0xFFFF;
  return p;
}
static bool blePrefsEqualLocal(const BlePktPrefs &p) {
  const BlePktPrefs l = blePrefsFromLocal();
  if ((p.flags & 0x07) != (l.flags & 0x07) || p.lubW != l.lubW || p.lubY != l.lubY ||
      p.qfeLow != l.qfeLow || p.qfeHigh != l.qfeHigh || p.gotoIdx != l.gotoIdx) return false;
  if ((p.flags & 0x08) && (!(l.flags & 0x08) || p.qfeRefCm != l.qfeRefCm)) return false;
  return true;
}
// Applica le impostazioni ricevute (dal master sullo slave, dallo slave sul master), salva e ridisegna.
static void bleApplyPrefs(const BlePktPrefs &p) {
  if (p.type != 11) return;
  if (altGsDragTarget != 0 || gpsSpeedDragTarget != 0 || qfeDragEdge != 0) return;   // sto trascinando io
  if (blePrefsEqualLocal(p)) return;
  const bool meters = (p.flags & 0x01) != 0, knots = (p.flags & 0x02) != 0, qfeFt = (p.flags & 0x04) != 0;
  const bool knotsChanged = knots != gpsSpeedShowKnots;
  if (p.gotoIdx < AIRFIELD_COUNT) { if (!navGotoActive || navGotoIdx != p.gotoIdx) navSetGoto(p.gotoIdx); }
  else navCancelGoto();
  altitudeUnitsIsMeters = meters;
  gpsSpeedShowKnots = knots;
  qfeShowFeet = qfeFt;
  if (p.lubW <= 3) lubberWidthIdx = p.lubW;
  if (p.lubY <= 2) lubberYellowIdx = p.lubY;
  const int lo = constrain((int)p.qfeLow, 0, 8980);
  qfeCircuitLowM = lo;
  qfeCircuitHighM = constrain((int)p.qfeHigh, lo + 20, 9000);
  if (p.flags & 0x08) {   // riferimento QFE (RESET QFE) dell'altro strumento
    qfeReferenceAltitudeMeters = p.qfeRefCm / 100.0f;
    qfeReferenceValid = true;
    qfeStartupResetDone = true;   // niente azzeramento automatico dopo: vale quello condiviso
    lastPrintedQfeMeters = -99999;
    lastPrintedQfeFeet = -99999;
  }
  preferences.begin("altimeter", false);
  preferences.putBool("is_meters", altitudeUnitsIsMeters);
  preferences.putBool("gs_kt", gpsSpeedShowKnots);
  preferences.putBool("qfe_ft", qfeShowFeet);
  preferences.putInt("lub_w", lubberWidthIdx);
  preferences.putInt("lub_y", lubberYellowIdx);
  preferences.putInt("qfe_cl", qfeCircuitLowM);
  preferences.putInt("qfe_ch", qfeCircuitHighM);
  preferences.end();
  lastGoToHomeXteState = -1;
  if (knotsChanged) { gsScaleNeedsErase = true; gsPointerDrawn = false; }
  qfeLastKey = 0xFFFFFFFFu;
  gpsSpeedLastKey = 0xFFFFFFFFu;
  if (currentScreen == MAIN_ALT_SCREEN || currentScreen == QFE_SCREEN || currentScreen == GPS_SPEED_SCREEN ||
      currentScreen == VARIOMETER_SCREEN || currentScreen == LUBBER_LINE_SCREEN ||
      currentScreen == GO_TO_HOME_SCREEN || currentScreen == NEAREST_FIELDS_SCREEN ||
      currentScreen == FIELDS_RADAR_SCREEN) redrawCurrentScreen();
}

// Master: copia i dati condivisi (mutex con timeout breve) e invia telemetria e stato volo.
static void bleMasterSend(bool sendGps) {
  if (bleChar == nullptr) return;
  BlePktGps g;
  BlePktSens s;
  if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(5)) != pdTRUE) return;  // meglio saltare un invio che bloccare
  g.type = 1;
  const unsigned long nowMs = millis();
  g.flags = (gpsDataValid ? 0x01 : 0x00)
          | ((gps_has_gps     && nowMs - gps_last_gsv_gps     < GSV_TIMEOUT_MS) ? 0x02 : 0)
          | ((gps_has_glonass && nowMs - gps_last_gsv_glonass < GSV_TIMEOUT_MS) ? 0x04 : 0)
          | ((gps_has_galileo && nowMs - gps_last_gsv_galileo < GSV_TIMEOUT_MS) ? 0x08 : 0)
          | ((gps_has_bds     && nowMs - gps_last_gsv_bds     < GSV_TIMEOUT_MS) ? 0x10 : 0)
          | ((gps_has_qzss    && nowMs - gps_last_gsv_qzss    < GSV_TIMEOUT_MS) ? 0x20 : 0);
  g.lat = gps_latitude;
  g.lon = gps_longitude;
  g.speed10 = (uint16_t)constrain(lroundf((float)gps_ground_speed_kmh * 10.0f), 0L, 65535L);
  g.course10 = (uint16_t)constrain(lroundf((float)gps_course * 10.0f), 0L, 3600L);
  g.altM = (int16_t)constrain(lroundf((float)gps_altitude), -32768L, 32767L);
  g.sats = (uint8_t)constrain((int)gps_satellites, 0, 255);
  g.h = (uint8_t)gps_utc_hours; g.m = (uint8_t)gps_utc_minutes; g.s = (uint8_t)gps_utc_seconds;
  s.type = 2;
  s.flags = (sharedAltitudeValid ? 0x01 : 0) | (sharedVarioValid ? 0x02 : 0) |
            (meteoValid ? 0x04 : 0) |   // 0x08 era il G-meter (rimosso)
            ((oatValid && !oatFromRemote && isfinite((float)oatTempC)) ? 0x10 : 0);   // sonda esterna: anche allo slave
  s.vario100 = (int16_t)constrain(lroundf(sharedVarioKalman * 100.0f), -32768L, 32767L);
  s.altCm = (int32_t)lroundf(sharedFilteredAlt * 100.0f);
  s.temp10 = (int16_t)constrain(lroundf((float)meteo_temperature_c * 10.0f), -32768L, 32767L);
  s.hum2 = (uint8_t)constrain(lroundf((float)meteo_humidity_percent * 2.0f), 0L, 255L);
  s.g100 = (s.flags & 0x10) ? (int16_t)lroundf((float)oatTempC * 10.0f) : 100;   // OAT nel posto del vecchio G-meter
  s.qnh10 = (uint16_t)constrain(lroundf((float)seaLevelPressure * 10.0f), 0L, 65535L);
  s.d = (uint8_t)gps_utc_day; s.mo = (uint8_t)gps_utc_month;
  s.y = (uint8_t)(gps_utc_year >= 2000 ? gps_utc_year - 2000 : 0);
  xSemaphoreGive(sensorDataMutex);

  if (sendGps) {
    bleChar->setValue((uint8_t *)&g, sizeof(g));
    bleNotify();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  bleChar->setValue((uint8_t *)&s, sizeof(s));
  bleNotify();

  BlePktTimer t;
  t.type = 3;
  t.running = automaticFlightRunning ? 1 : 0;
  unsigned long elapsed = automaticFlightAccumMillis;
  if (automaticFlightRunning) elapsed += millis() - automaticFlightStartMillis;
  t.elapsedMs = (uint32_t)elapsed;
  bleChar->setValue((uint8_t *)&t, sizeof(t));
  bleNotify();

  BlePktTimes tm; tm.type = 5;
  memcpy(tm.start, automaticStartUtc, 9); memcpy(tm.stop, automaticStopUtc, 9);
  bleChar->setValue((uint8_t *)&tm, sizeof(tm)); bleNotify();
  BlePktStatus st; st.type = 6;
  st.flags = (ictEnabled ? 0x01 : 0) | ((currentScreen == ICT_ALERT_SCREEN) ? 0x02 : 0) |
             ((currentScreen == FLIGHT_END_SCREEN) ? 0x04 : 0) | (demoMode ? 0x08 : 0);
  st.ictMinutes = (uint8_t)constrain(ictMinutes, 1, 30);
  st.ictElapsedMs = (uint32_t)(millis() - ictStartMillis);
  st.flightEndPrevious = (uint8_t)flightEndPreviousScreen;
  st.ictPrevious = (uint8_t)ictAlertPreviousScreen;
  st.tzIndex = (uint8_t)timezoneIndex;
  bleChar->setValue((uint8_t *)&st, sizeof(st)); bleNotify();
  static uint8_t homeTxDiv = 0;
  if (++homeTxDiv >= 5 && !demoHomeAutoSet) {   // 2 Hz; la home temporanea della demo non si invia
    homeTxDiv = 0;
    BlePktHome hp; hp.type = 8; hp.valid = homePositionValid ? 1 : 0;
    hp.lat7 = homePositionValid ? (int32_t)llround(homeLatitude * 1e7) : 0;
    hp.lon7 = homePositionValid ? (int32_t)llround(homeLongitude * 1e7) : 0;
    bleChar->setValue((uint8_t *)&hp, sizeof(hp)); bleNotify();
  }
  static uint8_t fuelTxDiv = 0;
  if (++fuelTxDiv >= 10) {   // 1 Hz: impostazioni e litri del master (i litri veri, mai quelli della demo)
    fuelTxDiv = 0;
    BlePktFuel fp; fp.type = 9; fp.tank = (uint8_t)fuelTankL; fp.lph2 = (uint8_t)lroundf(fuelRateLph * 2.0f);
    fp.resMin = (uint8_t)fuelReserveMin; fp.gnd2 = (uint8_t)lroundf(fuelGroundL * 2.0f); fp.full = 0;
    fp.liters10 = (uint16_t)lroundf(fuelLiters * 10.0f);
    bleChar->setValue((uint8_t *)&fp, sizeof(fp)); bleNotify();
    BlePktArc ap; ap.type = 10;   // soglie arco velocita' (ALTIMETER e GPS SPEED)
    ap.yel = (uint16_t)constrain(gsYellowKmh, 0, 500); ap.red = (uint16_t)constrain(gsRedKmh, 0, 500);
    bleChar->setValue((uint8_t *)&ap, sizeof(ap)); bleNotify();
    BlePktPrefs pp = blePrefsFromLocal();   // unita', QFE, corridoio
    bleChar->setValue((uint8_t *)&pp, sizeof(pp)); bleNotify();
    if (metFresh()) {   // meteo del telefono: lo slave non ce l'ha, il telefono parla solo col master
      BlePktMet mp; mp.type = 13;
      const int age = metAgeMin();
      mp.ageMin = (uint8_t)constrain(age < 0 ? 0 : age, 0, 255);
      mp.flags = (metQnhValid ? 0x01 : 0) | (metWindValid ? 0x02 : 0) | (metVisValid ? 0x04 : 0) |
                 (metPrecipValid ? 0x08 : 0) | (metFrzValid ? 0x10 : 0) | (metDewValid ? 0x20 : 0) |
                 (metAligned() ? 0x40 : 0);
      mp.dir2 = (uint8_t)constrain(metWindDir / 2, 0, 180);
      mp.qnh10 = (uint16_t)constrain(lroundf(metQnh * 10.0f), 0L, 65535L);
      mp.kt10 = (uint16_t)constrain(lroundf(metWindKt * 10.0f), 0L, 65535L);
      mp.gust10 = (uint16_t)constrain(lroundf(metGustKt * 10.0f), 0L, 65535L);
      mp.visM = (uint16_t)constrain(metVisM, 0, 65535);
      mp.precip10 = (uint8_t)constrain(lroundf(metPrecipMm * 10.0f), 0L, 255L);
      mp.prob = (uint8_t)constrain(metPrecipProb, 0, 100);
      mp.frzM = (uint16_t)constrain(metFrzM, 0, 65535);
      mp.dew10 = (int16_t)constrain(lroundf(metDewC * 10.0f), -32768L, 32767L);
      bleChar->setValue((uint8_t *)&mp, sizeof(mp)); bleNotify();
    }
  }
  if (slipBall.ready()) {   // sbandometro: l'accelerometro ce l'ha solo il master
    BlePktSlip sp; sp.type = 14;
    sp.flags = slipBall.gyroReady() ? 0x01 : 0;
    sp.g1000 = (int16_t)constrain(lroundf(slipBall.lateralG() * 1000.0f), -32768L, 32767L);
    sp.rate10 = (int16_t)constrain(lroundf(slipBall.turnRate() * 10.0f), -32768L, 32767L);
    bleChar->setValue((uint8_t *)&sp, sizeof(sp)); bleNotify();
  }
  if (bleAckKind != 0 && (long)(bleAckSendUntil - millis()) > 0) {   // avviso letto: ripetuto per 20 s
    const uint8_t ap[3] = { 12, bleAckSeq, bleAckKind };
    bleChar->setValue((uint8_t *)ap, sizeof(ap)); bleNotify();
  }
  if (bleLogTxPending) {
    BlePktLog lp; lp.type = 4; lp.index = bleLogTxIndex;
    const size_t offset = (size_t)bleLogTxIndex * 18;
    const size_t n = min((size_t)18, sizeof(FlightLog) - offset);
    memset(lp.data, 0, sizeof(lp.data));
    memcpy(lp.data, ((const uint8_t *)&bleOutgoingFlightLog) + offset, n);
    bleChar->setValue((uint8_t *)&lp, sizeof(lp)); bleNotify();
    if (++bleLogTxIndex >= (sizeof(FlightLog) + 17) / 18) {
      bleLogTxIndex = 0;
      if (++bleLogTxPass >= 2) { bleLogTxPass = 0; bleLogTxPending = false; }
    }
  }
}

static bool bleStackStart(uint8_t role) {
  const size_t freeInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  AVEA_LOGF("[BLE] avvio ruolo %u: RAM interna libera %u byte (minimo %u)\n",
                (unsigned)role, (unsigned)freeInt, (unsigned)BLE_MIN_FREE_INTERNAL);
  if (freeInt < BLE_MIN_FREE_INTERNAL) {
    AVEA_LOGLN("[BLE] RAM interna insufficiente: BLE non avviato");
    return false;
  }
  BLEDevice::init(role == BLE_ROLE_MASTER ? "AVEA-M" : "AVEA-S");
  if (role == BLE_ROLE_MASTER) {
    bleServer = BLEDevice::createServer();
    bleServer->setCallbacks(&bleServerCb);
    BLEService *svc = bleServer->createService(BLE_SVC_UUID);
    bleChar = svc->createCharacteristic(BLE_CHR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    bleChar->addDescriptor(new BLE2902());
    bleCmdChar = svc->createCharacteristic(BLE_CMD_UUID,
                   BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    bleCmdChar->setCallbacks(&bleCmdCb);
    // Caratteristica del meteo: ci scrive il telefono (app AVEA Link), non lo slave.
    bleMetChar = svc->createCharacteristic(BLE_MET_UUID,
                   BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    bleMetChar->setCallbacks(&bleMetCb);
    BLEDevice::setMTU(64);   // il pacchetto standard sta in 20 byte: col meteo e la posizione ne servono 28
    bleDstChar = svc->createCharacteristic(BLE_DST_UUID, BLECharacteristic::PROPERTY_READ);
    bleUpdateDestination();
    svc->start();
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(BLE_SVC_UUID);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();
  } else {
    bleCli = BLEDevice::createClient();
    bleCli->setClientCallbacks(&bleClientCb);
    BLEScan *scan = BLEDevice::getScan();
    scan->setAdvertisedDeviceCallbacks(&bleScanCb, false);
    scan->setActiveScan(false);
    scan->setInterval(100);
    scan->setWindow(99);
  }
  bleLinkUp = false;
  bleNeedAdv = false;
  bleStackUp = true;
  AVEA_LOGF("[BLE] avviato: RAM interna libera %u byte\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
  return true;
}

static void bleStackStop() {
  if (!bleStackUp) return;
  if (bleCli != nullptr && bleCli->isConnected()) bleCli->disconnect();
  if (bleFoundDev) { delete bleFoundDev; bleFoundDev = nullptr; }
  bleCli = nullptr;
  bleServer = nullptr;
  bleChar = nullptr;
  bleCmdChar = nullptr;
  bleCmdRemote = nullptr;
  BLEDevice::deinit(false);   // false: la memoria resta riservata al BLE, cosi' si puo' riavviare senza reset
  bleStackUp = false;
  bleLinkUp = false;
  bleConnCount = 0;
  AVEA_LOGLN("[BLE] fermato");
}

// Slave: ricerca del master (bloccante, ma nel task BLE), connessione e iscrizione alle notifiche.
static void bleSlaveConnect() {
  bleFound = false;
  bleCmdRemote = nullptr;
  BLEScan *scan = BLEDevice::getScan();
  scan->start(3, false);
  scan->clearResults();
  if (!bleFound || bleFoundDev == nullptr || bleCli == nullptr) return;
  const bool ok = bleCli->connect(bleFoundDev);
  delete bleFoundDev;
  bleFoundDev = nullptr;
  if (!ok) { AVEA_LOGLN("[BLE] connessione al master fallita"); return; }
  BLERemoteService *svc = bleCli->getService(BLEUUID(BLE_SVC_UUID));
  BLERemoteCharacteristic *ch = svc ? svc->getCharacteristic(BLEUUID(BLE_CHR_UUID)) : nullptr;
  if (ch != nullptr && ch->canNotify()) {
    BLERemoteCharacteristic *cc = svc->getCharacteristic(BLEUUID(BLE_CMD_UUID));
    bleCmdRemote = (cc != nullptr && cc->canWrite()) ? cc : nullptr;   // nullptr se il master ha un firmware piu' vecchio
    ch->registerForNotify(bleNotifyCb);
    bleLinkUp = true;
    AVEA_LOGLN("[BLE] collegato al master");
  } else {
    AVEA_LOGLN("[BLE] servizio dati non trovato");
    bleCli->disconnect();
  }
}

// Slave: invia al master i comandi in coda (chiamata dal task BLE, mai dal loop()).
static void bleSlaveSendCommands() {
  uint8_t bits, tz; uint16_t q10, ict, arcVal; bool demoOn, arc, slipZero;
  BlePktHome hp; hp.type = 8;
  BlePktFuel fp;
  portENTER_CRITICAL(&bleMux);
  bits = bleCmdTxBits; q10 = bleCmdTxQnh10; ict = bleCmdTxIct; demoOn = bleCmdTxDemoOn; tz = bleCmdTxTz; bleCmdTxBits = 0;
  arc = bleCmdTxArc; arcVal = bleCmdTxArcVal; bleCmdTxArc = false;
  slipZero = bleCmdTxSlipZero; bleCmdTxSlipZero = false;
  const bool prefsNew = bleCmdTxPrefsNew; const BlePktPrefs prefsPkt = bleCmdTxPrefs; bleCmdTxPrefsNew = false;
  fp = bleCmdTxFuel;
  hp.valid = bleCmdTxHomeValid ? 1 : 0; hp.lat7 = bleCmdTxHomeLat7; hp.lon7 = bleCmdTxHomeLon7;
  portEXIT_CRITICAL(&bleMux);
  uint8_t ackKind = 0;
  if (bleAckTxKind != 0) {
    portENTER_CRITICAL(&bleMux);
    if (millis() - bleAckTxAt < 20000UL) ackKind = bleAckTxKind;   // tocco fatto prima del collegamento: vale 20 s
    bleAckTxKind = 0;
    portEXIT_CRITICAL(&bleMux);
  }
  if ((bits == 0 && !arc && !slipZero && !prefsNew && ackKind == 0) || bleCmdRemote == nullptr) return;
  BlePktCmd p; p.type = 7;
  if (slipZero) { p.cmd = 10; p.value = 0; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (ackKind) { p.cmd = 9; p.value = ackKind; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (arc) { p.cmd = 8; p.value = arcVal; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (prefsNew) bleCmdRemote->writeValue((uint8_t *)&prefsPkt, sizeof(prefsPkt), true);
  if (bits & 0x01) { p.cmd = 1; p.value = 0; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (bits & 0x02) { p.cmd = 2; p.value = q10; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (bits & 0x04) { p.cmd = demoOn ? 3 : 4; p.value = 0; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (bits & 0x08) { p.cmd = 5; p.value = tz; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (bits & 0x10) { p.cmd = 6; p.value = ict; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (bits & 0x20) { p.cmd = 7; p.value = 0; bleCmdRemote->writeValue((uint8_t *)&p, sizeof(p), true); }
  if (bits & 0x40) bleCmdRemote->writeValue((uint8_t *)&hp, sizeof(hp), true);
  if (bits & 0x80) bleCmdRemote->writeValue((uint8_t *)&fp, sizeof(fp), true);
}

static void bleTask(void *) {
  uint8_t runningRole = BLE_ROLE_OFF;
  unsigned long retryAt = 0;
  vTaskDelay(pdMS_TO_TICKS(3000));   // lascia finire l'avvio prima di prendere RAM per il BLE
  TickType_t xLast = xTaskGetTickCount();
  for (;;) {
    const uint8_t want = bleWifiHold ? (uint8_t)BLE_ROLE_OFF : bleRole;
    if (want != runningRole) {
      bleStackStop();
      runningRole = BLE_ROLE_OFF;
      if (want == BLE_ROLE_OFF) {
        bleState = (bleWifiHold && bleRole != BLE_ROLE_OFF) ? BLE_ST_PAUSED : BLE_ST_OFF;
      } else if (millis() >= retryAt) {
        if (bleStackStart(want)) {
          runningRole = want;
          bleState = BLE_ST_WAIT;
        } else {
          bleState = BLE_ST_ERROR;
          retryAt = millis() + 10000UL;   // niente tentativi a raffica
        }
      }
    }

    if (runningRole == BLE_ROLE_MASTER) {
      if (bleNeedAdv) {
        bleNeedAdv = false;
        vTaskDelay(pdMS_TO_TICKS(200));
        BLEDevice::startAdvertising();
      }
      if (bleLinkUp) {
        bleState = BLE_ST_LINKED;
        bleMasterSend(true);                 // GPS e sensori a 10 Hz - bleMasterSend((tick++ & 1) == 0);   // sensori 10 Hz, GPS 5 Hz
      } else {
        bleState = BLE_ST_WAIT;
      }
      vTaskDelayUntil(&xLast, pdMS_TO_TICKS(100));
    } else if (runningRole == BLE_ROLE_SLAVE) {
      if (bleLinkUp && bleCli != nullptr && bleCli->isConnected()) {
        bleState = BLE_ST_LINKED;
        bleSlaveSendCommands();
        vTaskDelayUntil(&xLast, pdMS_TO_TICKS(100));
      } else {
        bleLinkUp = false;
        bleState = BLE_ST_WAIT;
        bleSlaveConnect();
        if (!bleLinkUp) vTaskDelay(pdMS_TO_TICKS(500));
        xLast = xTaskGetTickCount();
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(200));
      xLast = xTaskGetTickCount();
    }
  }
}

void bleStartTask() {
  xTaskCreatePinnedToCore(bleTask, "BleTask", 6144, NULL, 1, NULL, 0);  // priorita' sotto SensorTask (2)
}

void bleSetRole(uint8_t role) {
  if (role > BLE_ROLE_SLAVE) role = BLE_ROLE_OFF;
  bleRole = role;
  preferences.begin("altimeter", false);
  preferences.putUChar("ble_role", role);
  preferences.end();
}

void bleCycleRole() {
  if (flightTimerRunning || automaticFlightRunning) {   // niente init/deinit dello stack BLE durante il volo
    playTone(500, 100);
    return;
  }
  bleSetRole((uint8_t)((bleRole + 1) % 3));
  playTone(1200, 50);
}

// Il BLE NON si ferma piu' ad ogni cambio pagina: prima il passaggio dalla pagina WIFI LOGS (adiacente
// a GPS INFO) spegneva lo stack BLE e lo slave perdeva il collegamento. Ora il BLE resta attivo insieme
// al WiFi; si ferma solo se la RAM interna e' davvero troppo poca per far partire anche il WiFi.
#define BLE_WIFI_MIN_FREE_INTERNAL 70000UL
void bleSuspendForWifi() {
  if (bleRole == BLE_ROLE_OFF || !bleStackUp) { bleWifiHold = false; return; }
  const size_t freeInt = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  if (freeInt >= BLE_WIFI_MIN_FREE_INTERNAL) {
    AVEA_LOGF("[BLE] resta attivo con il WiFi (RAM interna libera %u byte)\n", (unsigned)freeInt);
    bleWifiHold = false;
    return;
  }
  AVEA_LOGF("[BLE] RAM interna bassa (%u byte): BLE in pausa mentre il WiFi e' attivo\n", (unsigned)freeInt);
  bleWifiHold = true;
  const unsigned long t0 = millis();
  while (bleStackUp && millis() - t0 < 2500UL) {   // attende che il task BLE liberi la RAM prima del WiFi
    esp_task_wdt_reset();
    delay(20);
  }
}

void bleResumeAfterWifi() {
  bleWifiHold = false;
}

// ---- Simulazione (demo): usata dal tocco locale e dai comandi BLE dello slave ----
bool startDemoMode(unsigned long nowMs) {
  if (demoMode) return true;
  if (flightTimerRunning || automaticFlightRunning) {
    // Non sostituire i dati reali mentre e' in corso un volo.
    playTone(500, 100);
    return false;
  }
  // Se la home non e' impostata, usa DEMO_BASE come home temporanea per la demo.
  // Al termine della demo viene ripristinato lo stato precedente (non si tocca NVS).
  if (!homePositionValid) {
    homeLatitude = DEMO_BASE_LAT;
    homeLongitude = DEMO_BASE_LON;
    homePositionValid = true;
    demoHomeAutoSet = true;
    lubberLegValid = false;   // la corda si ricalcola dal punto di decollo
  }
  ictMinutesBeforeDemo = ictMinutes;
  ictEnabledBeforeDemo = ictEnabled;
  demoStartMillis = nowMs;
  demoSunsetClockInitialized = false;
  demoSunsetShown = false;
  demoSunsetClockStartMillis = 0;
  demoIctInitialized = false;
  demoRiskTourActive = false;
  demoRiskTourDone = false;
  demoIntroRequestMillis = nowMs;
  demoIntroActive = true;   // prima la sequenza visibility / ghiaccio / sunset, poi la routine demo
  demoMode = true;
  demoTourStart();          // sketch_test: rotazione di tutte le pagine e avvisi (sostituisce la sequenza iniziale)
  if (currentScreen == FIRMWARE_SCREEN) drawFirmwareScreen();
  playTone(1500, 80);
  return true;
}

void stopDemoMode(unsigned long nowMs) {
  if (!demoMode) return;
  // Se la demo viene disattivata durante un volo simulato,
  // aggiorna e salva i dati raccolti fino a questo momento.
  if (automaticFlightRunning) {
    updateFlightLogMetrics(nowMs, (float)gps_ground_speed_kmh, gpsDataValid);
    automaticFlightAccumMillis += nowMs - automaticFlightStartMillis;
    automaticFlightRunning = false;
    flightStatsLastUpdateMillis = 0;
    getLocalTimeText(automaticStopUtc, sizeof(automaticStopUtc));
    onFlightEnd();
  }
  speedAboveStartSince = 0;
  speedBelowStopSince = 0;
  gpsInvalidFlightSince = 0;
  demoTourStop();
  demoMode = false;
  // Meteo finto della demo buttato via: senza questo, per mezz'ora dopo la demo lo strumento
  // mostrerebbe QNH e vento inventati come se arrivassero dal telefono, e durante la demo
  // avrebbe coperto quelli veri.
  metQnhValid = false; metWindValid = false;
  metVisValid = false; metPrecipValid = false; metFrzValid = false; metDewValid = false;
  metPhonePosValid = false;
  metAtMs = 0;
  metDestChangedMs = 0; metAlignedAtMs = 0; metWxAutoPending = false; wxAutoUntilMs = 0;
  // Se la home era stata impostata automaticamente dalla demo, la si rilascia
  // (l'utente non l'aveva mai salvata in NVS).
  if (demoHomeAutoSet) {
    homePositionValid = false;
    homeLatitude = 0.0;
    homeLongitude = 0.0;
    demoHomeAutoSet = false;
    lubberLegValid = false;
  }
  ictMinutes = ictMinutesBeforeDemo;
  ictEnabled = ictEnabledBeforeDemo;
  ictStartMillis = nowMs;
  demoStartMillis = 0;
  demoSunsetClockInitialized = false;
  demoSunsetShown = false;
  demoSunsetClockStartMillis = 0;
  demoIctInitialized = false;
  if (currentScreen == FIRMWARE_SCREEN) drawFirmwareScreen();
  playTone(900, 80);
}

// ---- QNH: applicazione locale (usata dal comando dello slave sul master e dal valore del master sullo slave) ----
static void bleSetQnhLocal(uint16_t q10) {
  const float hpa = constrain(q10 / 10.0f, QNH_MIN_HPA, QNH_MAX_HPA);
  seaLevelPressure = hpa;
  preferences.begin("altimeter", false);
  preferences.putFloat("qnh", hpa);
  preferences.end();
  qnhChangedAtMillis = millis();
  if (currentScreen == QNH_SETUP_SCREEN) drawQnhSetupScreen();
}

// ---- Fuso UTC dell'orologio GPS: applicazione locale (comando dello slave sul master / valore del master sullo slave) ----
static void bleSetTimezoneLocal(uint8_t idx) {
  if (idx > 4) return;
  timezoneIndex = idx;
  preferences.begin("altimeter", false);
  preferences.putInt("tz_idx", timezoneIndex);
  preferences.end();
  lastPrintedClockKey = 0xFFFFFFFFUL;
  if (currentScreen == GPS_CLOCK_SCREEN) drawGpsClockScreen();
}

// ---- Slave: accodamento comandi verso il master (inviati dal task BLE) ----
// Se il collegamento non e' attivo il comando viene scartato: non deve scattare in ritardo alla riconnessione.
void bleQueueCloseFlightEnd() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxBits |= 0x01;
  portEXIT_CRITICAL(&bleMux);
}
// Slave: chiede al master di azzerare lo sbandometro (la pallina mostrata e' la sua)
void bleQueueSlipZero() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxSlipZero = true;
  portEXIT_CRITICAL(&bleMux);
}
void bleQueueQnh(float hpa) {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxQnh10 = (uint16_t)constrain(lroundf(hpa * 10.0f), 0L, 65535L);
  bleCmdTxBits |= 0x02;
  bleQnhLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}
void bleQueueDemo(bool on) {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxDemoOn = on;
  bleCmdTxBits |= 0x04;
  portEXIT_CRITICAL(&bleMux);
}
void bleQueueIct() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxIct = (uint16_t)(constrain(ictMinutes, 1, 30) | (ictEnabled ? 0x80 : 0));
  bleCmdTxBits |= 0x10;
  bleIctLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}
// Slave: invia al master la home appena impostata (SET HOME o COORDS); il master la applica e la rinvia.
void bleQueueHome() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxHomeValid = homePositionValid;
  bleCmdTxHomeLat7 = homePositionValid ? (int32_t)llround(homeLatitude * 1e7) : 0;
  bleCmdTxHomeLon7 = homePositionValid ? (int32_t)llround(homeLongitude * 1e7) : 0;
  bleCmdTxBits |= 0x40;
  bleHomeLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}
// Slave: invia al master i valori FUEL appena salvati; il master li applica e li rinvia a tutti.
void bleQueueFuel(bool fullPressed) {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxFuel.type = 9; bleCmdTxFuel.tank = (uint8_t)fuelTankL; bleCmdTxFuel.lph2 = (uint8_t)lroundf(fuelRateLph * 2.0f);
  bleCmdTxFuel.resMin = (uint8_t)fuelReserveMin; bleCmdTxFuel.gnd2 = (uint8_t)lroundf(fuelGroundL * 2.0f);
  bleCmdTxFuel.full = fullPressed ? 1 : 0; bleCmdTxFuel.liters10 = (uint16_t)lroundf(fuelLiters * 10.0f);
  bleCmdTxBits |= 0x80;
  bleFuelLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}
// Slave: invia al master le soglie dell'arco velocita' appena regolate; il master le applica e le rinvia a tutti.
void bleQueueArc() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxArcVal = (uint16_t)((constrain(gsYellowKmh, 0, 511) & 0x1FF) | ((constrain(gsRedKmh - gsYellowKmh, 0, 127) & 0x7F) << 9));
  bleCmdTxArc = true;
  bleArcLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}
// Slave: invia al master le impostazioni condivise appena cambiate qui; il master le applica e le rinvia.
void bleQueuePrefs() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  const BlePktPrefs p = blePrefsFromLocal();
  portENTER_CRITICAL(&bleMux);
  bleCmdTxPrefs = p;
  bleCmdTxPrefsNew = true;
  blePrefsLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}
void bleQueueIctDismiss() {
  if (bleRole != BLE_ROLE_SLAVE || bleState != BLE_ST_LINKED) return;
  portENTER_CRITICAL(&bleMux);
  bleCmdTxBits |= 0x20;
  bleIctLocalAt = millis();
  portEXIT_CRITICAL(&bleMux);
}

// Master: esegue i comandi arrivati dallo slave (chiamata dal loop()).
static void bleApplyRemoteCommands() {
  if (bleRole != BLE_ROLE_MASTER) return;
  uint8_t bits, tz; uint16_t q10, ictValue; bool demoOn;
  bool homeValid; int32_t homeLat7, homeLon7;
  BlePktFuel fuelPkt;
  portENTER_CRITICAL(&bleMux);
  fuelPkt = bleCmdRxFuel;
  bits = bleCmdRxBits; q10 = bleCmdRxQnh10; ictValue = bleCmdRxIct; demoOn = bleCmdRxDemoOn; tz = bleCmdRxTz; bleCmdRxBits = 0;
  homeValid = bleCmdRxHomeValid; homeLat7 = bleCmdRxHomeLat7; homeLon7 = bleCmdRxHomeLon7;
  const bool arc = bleCmdRxArc; const uint16_t arcVal = bleCmdRxArcVal; bleCmdRxArc = false;
  const bool prefsNew = bleCmdRxPrefsNew; const BlePktPrefs prefsPkt = bleCmdRxPrefs; bleCmdRxPrefsNew = false;
  const bool slipZero = bleCmdRxSlipZero; bleCmdRxSlipZero = false;
  portEXIT_CRITICAL(&bleMux);
  if (slipZero && slipBall.ready() && !slipBall.isRemote()) {   // doppio tocco sull'ampolla dello slave
    slipBall.setZero();
    if (currentScreen == SLIP_SCREEN) drawSlipScreen(true);
  }
  if (arc) arcApplyShared(arcVal & 0x1FF, (arcVal & 0x1FF) + ((arcVal >> 9) & 0x7F));   // soglie arco velocita' dallo slave
  if (prefsNew) bleApplyPrefs(prefsPkt);                            // unita', QFE, corridoio dallo slave
  if (bits == 0) return;
  if (bits & 0x40) applyHomeLocal(homeValid, homeLat7 / 1e7, homeLon7 / 1e7);   // home dallo slave
  if (bits & 0x80) {   // FUEL SETUP salvato sullo slave
    fuelApplyShared(fuelPkt.tank, fuelPkt.lph2 / 2.0f, fuelPkt.resMin, fuelPkt.gnd2 / 2.0f,
                    fuelPkt.liters10 / 10.0f, fuelPkt.full != 0);
  }
  if (bits & 0x01) {
    // Chiude la schermata di fine volo sul master: lo stato (bit 0x04) sparisce e anche lo slave si chiude.
    if (currentScreen == FLIGHT_END_SCREEN) dismissFlightEndScreen();
  }
  if (bits & 0x02) bleSetQnhLocal(q10);
  if (bits & 0x10) {
    const int minutes = constrain((int)(ictValue & 0x1F), 1, 30);
    const bool enabled = (ictValue & 0x80) != 0;
    if (ictMinutes != minutes || ictEnabled != enabled) {
      ictMinutes = minutes; ictEnabled = enabled; ictStartMillis = millis();
      preferences.begin("altimeter", false);
      preferences.putInt("ict_minutes", ictMinutes); preferences.putBool("ict_enabled", ictEnabled);
      preferences.end();
      if (currentScreen == ICT_SCREEN) drawIctScreen();
    }
  }
  if ((bits & 0x20) && currentScreen == ICT_ALERT_SCREEN) dismissIctAlertScreen();
  if (bits & 0x08) bleSetTimezoneLocal(tz);
  if (bits & 0x04) {
    if (demoOn && !demoMode) startDemoMode(millis());
    else if (!demoOn && demoMode) stopDemoMode(millis());
  }
}

// Slave: il QNH impostato sul master viene copiato qui. Si agisce solo quando il valore ricevuto CAMBIA
// (e al primo pacchetto dopo il collegamento), cosi' un QNH appena impostato sullo slave non viene
// sovrascritto dal vecchio valore del master prima che questo abbia applicato il comando.
static void bleApplyRemoteQnh() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  BleRemote r;
  portENTER_CRITICAL(&bleMux);
  r = bleRemote;
  portEXIT_CRITICAL(&bleMux);
  static int lastRemoteQnh10 = -1;
  if (bleState != BLE_ST_LINKED) { lastRemoteQnh10 = -1; return; }
  if (r.sensAtMs == 0 || millis() - r.sensAtMs > BLE_DATA_TIMEOUT_MS) return;
  const int q10 = (int)lroundf(r.qnh * 10.0f);
  if (q10 == lastRemoteQnh10) return;
  lastRemoteQnh10 = q10;
  if (millis() - bleQnhLocalAt < 1500UL) return;   // sto modificando io: il master sta ancora applicando i miei valori
  if (q10 < (int)(QNH_MIN_HPA * 10.0f) || q10 > (int)(QNH_MAX_HPA * 10.0f)) return;
  if (q10 == (int)lroundf((float)seaLevelPressure * 10.0f)) return;
  bleSetQnhLocal((uint16_t)q10);
}

// Slave: il fuso UTC impostato sul master viene copiato qui (stessa logica a "cambio di valore" del QNH).
// Slave: la home del master viene copiata qui (stessa logica a "cambio di valore" del QNH).
static void bleApplyRemoteHome() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  BleRemote r;
  portENTER_CRITICAL(&bleMux);
  r = bleRemote;
  portEXIT_CRITICAL(&bleMux);
  static bool lastValid = false;
  static int32_t lastLat7 = 0, lastLon7 = 0;
  static bool haveLast = false;
  if (bleState != BLE_ST_LINKED) { haveLast = false; return; }
  if (r.homeAtMs == 0 || millis() - r.homeAtMs > BLE_DATA_TIMEOUT_MS) return;
  if (haveLast && r.homeValid == lastValid && r.homeLat7 == lastLat7 && r.homeLon7 == lastLon7) return;
  haveLast = true; lastValid = r.homeValid; lastLat7 = r.homeLat7; lastLon7 = r.homeLon7;
  if (millis() - bleHomeLocalAt < 3000UL) return;   // sto impostando io: il master sta ancora applicando la mia home
  const int32_t myLat7 = homePositionValid ? (int32_t)llround(homeLatitude * 1e7) : 0;
  const int32_t myLon7 = homePositionValid ? (int32_t)llround(homeLongitude * 1e7) : 0;
  if (r.homeValid == homePositionValid && r.homeLat7 == myLat7 && r.homeLon7 == myLon7 && !demoHomeAutoSet) return;
  if (!r.homeValid && demoHomeAutoSet) return;   // home temporanea della demo: la lascio
  if (!r.homeValid && homePositionValid) { bleQueueHome(); return; }   // il master non ha la home: gli mando la mia
  applyHomeLocal(r.homeValid, r.homeLat7 / 1e7, r.homeLon7 / 1e7);
}

// Slave: meteo del telefono e sbandometro, che arrivano dal master perche' qui non c'e' ne' il
// telefono (il servizio Bluetooth esiste solo sul master) ne' l'accelerometro.
static void bleApplyRemoteMetSlip() {
  if (bleRole != BLE_ROLE_SLAVE || demoMode) return;
  BlePktMet m; BlePktSlip s; unsigned long mAt, sAt;
  portENTER_CRITICAL(&bleMux);
  m = bleRemote.met; mAt = bleRemote.metAtMs;
  s = bleRemote.slip; sAt = bleRemote.slipAtMs;
  portEXIT_CRITICAL(&bleMux);
  const unsigned long now = millis();
  const bool linked = (bleState == BLE_ST_LINKED);

  // --- meteo ---
  static unsigned long lastMetAt = 0;
  if (linked && mAt != 0 && mAt != lastMetAt && now - mAt <= BLE_DATA_TIMEOUT_MS && m.type == 13) {
    lastMetAt = mAt;
    metQnhValid = (m.flags & 0x01) != 0;
    metWindValid = (m.flags & 0x02) != 0;
    metVisValid = (m.flags & 0x04) != 0;
    metPrecipValid = (m.flags & 0x08) != 0;
    metFrzValid = (m.flags & 0x10) != 0;
    metDewValid = (m.flags & 0x20) != 0;
    metAlignedRemote = (m.flags & 0x40) != 0;
    metQnh = m.qnh10 / 10.0f;
    metWindDir = (int)m.dir2 * 2;
    metWindKt = m.kt10 / 10.0f;
    metGustKt = m.gust10 / 10.0f;
    metVisM = (int)m.visM;
    metPrecipMm = m.precip10 / 10.0f;
    metPrecipProb = (int)m.prob;
    metFrzM = (int)m.frzM;
    metDewC = m.dew10 / 10.0f;
    // Il dato e' arrivato adesso ma era gia' vecchio di tanti minuti: l'eta' in piu' si tiene da
    // parte, altrimenti la pagina scriverebbe "0 min" su un meteo di venti minuti prima.
    metAtMs = now;
    metExtraAgeMs = (unsigned long)m.ageMin * 60000UL;
    // Il master dice che il meteo si e' riallineato al campo nuovo: anche qui il pallino pulsa e
    // a terra la pagina si mostra da sola, come succede sul master.
    static bool prevAligned = true;
    const bool nowAligned = (m.flags & 0x40) != 0;
    if (nowAligned && !prevAligned) { metAlignedAtMs = now; metWxAutoPending = true; }
    prevAligned = nowAligned;
  } else if (lastMetAt != 0 && (!linked || now - mAt > BLE_DATA_TIMEOUT_MS)) {
    lastMetAt = 0;                 // collegamento perso: il meteo invecchia da solo e sparisce
    metQnhValid = metWindValid = metVisValid = false;
    metPrecipValid = metFrzValid = metDewValid = false;
  }

  // --- sbandometro ---
  static bool slipSeen = false;
  if (linked && sAt != 0 && s.type == 14) {
    if (!slipSeen) slipSeen = true;
    const bool fresh = (now - sAt) <= 2000UL;
    slipBall.setRemote(true, fresh && (s.flags & 0x01), s.g1000 / 1000.0f, s.rate10 / 10.0f);
  } else if (slipSeen) {
    slipBall.setRemote(true, false, 0.0f, 0.0f);   // la pagina resta, ma dice che il dato non c'e'
  }
}

// Slave: impostazioni e litri del master copiati qui (il master e' il riferimento, anche in volo).
static void bleApplyRemoteFuel() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  BlePktFuel f; unsigned long at;
  portENTER_CRITICAL(&bleMux);
  f = bleRemote.fuel; at = bleRemote.fuelAtMs;
  portEXIT_CRITICAL(&bleMux);
  static unsigned long lastAt = 0;
  if (bleState != BLE_ST_LINKED || at == 0 || at == lastAt || millis() - at > BLE_DATA_TIMEOUT_MS) return;
  lastAt = at;
  if (f.type != 9 || f.tank < 5) return;
  if (millis() - bleFuelLocalAt < 3000UL) return;   // ho appena salvato io: il master sta ancora applicando
  if (currentScreen == FUEL_SETUP_SCREEN) return;   // non cambiare i valori mentre li sto impostando
  fuelApplyShared(f.tank, f.lph2 / 2.0f, f.resMin, f.gnd2 / 2.0f, f.liters10 / 10.0f, false);
}

static void bleApplyRemoteTimezone() {
  return;   // fuso automatico dalla posizione su ogni strumento: niente piu' copia del fuso dal master
  if (bleRole != BLE_ROLE_SLAVE) return;
  BleRemote r;
  portENTER_CRITICAL(&bleMux);
  r = bleRemote;
  portEXIT_CRITICAL(&bleMux);
  static int lastRemoteTz = -1;
  if (bleState != BLE_ST_LINKED) { lastRemoteTz = -1; return; }
  if (r.statusAtMs == 0 || millis() - r.statusAtMs > BLE_DATA_TIMEOUT_MS) return;
  if ((int)r.remoteTzIndex == lastRemoteTz) return;
  lastRemoteTz = (int)r.remoteTzIndex;
  if (millis() - bleTzLocalAt < 1500UL) return;   // sto cambiando io: il master sta ancora applicando il mio valore
  if (r.remoteTzIndex > 4 || (int)r.remoteTzIndex == timezoneIndex) return;
  bleSetTimezoneLocal(r.remoteTzIndex);
}

// Slave: le soglie dell'arco velocita' del master vengono copiate qui (logica a "cambio di valore" del QNH).
static void bleApplyRemoteArc() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  uint16_t yel, red; unsigned long at;
  portENTER_CRITICAL(&bleMux);
  yel = bleRemote.arcYel; red = bleRemote.arcRed; at = bleRemote.arcAtMs;
  portEXIT_CRITICAL(&bleMux);
  static unsigned long lastAt = 0;
  if (bleState != BLE_ST_LINKED || at == 0 || at == lastAt) return;
  if (millis() - at > BLE_DATA_TIMEOUT_MS * 2) return;   // inviate a 1 Hz
  lastAt = at;
  if (millis() - bleArcLocalAt < 3000UL) return;   // ho appena regolato io: il master sta ancora applicando
  if (yel < 20 || red <= yel) return;
  arcApplyShared(yel, red);   // non fa nulla se sono gia' uguali o se sto trascinando (riprova al pacchetto dopo)
}

// Slave: impostazioni condivise del master copiate qui (il master e' il riferimento). Se differiscono
// dalle mie le applico, tranne nei 3 s dopo una mia modifica (il master la sta ancora applicando).
static void bleApplyRemotePrefs() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  BlePktPrefs p; unsigned long at;
  portENTER_CRITICAL(&bleMux);
  p = bleRemote.prefs; at = bleRemote.prefsAtMs;
  portEXIT_CRITICAL(&bleMux);
  static unsigned long lastAt = 0;
  if (bleState != BLE_ST_LINKED || at == 0 || at == lastAt || millis() - at > BLE_DATA_TIMEOUT_MS * 2) return;
  lastAt = at;
  if (millis() - blePrefsLocalAt < 3000UL) return;
  if (currentScreen == QNH_SETUP_SCREEN) return;   // non cambiare METERS/FEET mentre li sto scegliendo
  bleApplyPrefs(p);
}

// Slave: GPS e sensori arrivano dal master. Chiamata dal loop() subito dopo il parsing NMEA,
// quindi senza concorrenza col parser. Non blocca mai (mutex con timeout 0).
void bleApplyRemoteTimer() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  const unsigned long now = millis();
  BleRemote r;
  portENTER_CRITICAL(&bleMux);
  r = bleRemote;
  portEXIT_CRITICAL(&bleMux);
  if (bleState != BLE_ST_LINKED || r.timerAtMs == 0 ||
      now - r.timerAtMs > BLE_DATA_TIMEOUT_MS) return;
  const bool wasRunning = automaticFlightRunning;
  automaticFlightRunning = r.automaticRunning;
  automaticFlightAccumMillis = r.automaticElapsedMs;
  if (automaticFlightRunning) automaticFlightStartMillis = now;
  else if (wasRunning) { flightStatsLastUpdateMillis = 0; onFlightEnd(); }
}
static void bleApplyRemoteLog() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  FlightLog incoming; bool ready = false;
  portENTER_CRITICAL(&bleMux);
  if (bleRemote.remoteLogReady) {
    memcpy(&incoming, bleRemote.remoteLogBytes, sizeof(incoming));
    bleRemote.remoteLogReady = false; bleRemote.remoteLogMask = 0; ready = true;
  }
  portEXIT_CRITICAL(&bleMux);
  if (!ready) return;
  sanitizeFlightLog(incoming);
  strncpy(automaticStartUtc, incoming.startTime, sizeof(automaticStartUtc)-1);
  automaticStartUtc[sizeof(automaticStartUtc)-1] = '\0';
  strncpy(automaticStopUtc, incoming.stopTime, sizeof(automaticStopUtc)-1);
  automaticStopUtc[sizeof(automaticStopUtc)-1] = '\0';
  strncpy(flightDate, incoming.date, sizeof(flightDate)-1);
  flightDate[sizeof(flightDate)-1] = '\0';
  automaticFlightAccumMillis = incoming.durationSec * 1000UL;
  flightMaxAltitudeMeters = incoming.maxAltitude; flightMaxClimbRate = incoming.maxClimb;
  flightFuelUsedL = incoming.fuelUsed; flightMaxGpsSpeedKmh = incoming.maxSpeed;
  flightTotalDistanceKm = incoming.distance; flightMinTempC = incoming.minTemp;
  flightMaxTempC = incoming.maxTemp; flightMinHumidity = incoming.minHumidity;
  flightMaxHumidity = incoming.maxHumidity; flightMinG = incoming.minG; flightMaxG = incoming.maxG;
  flightEndTempC = incoming.endTemp; flightEndHumidity = incoming.endHumidity;
  flightEndEnvironmentValid = incoming.endEnvironmentValid;
  flightLogMetricsStarted = true; flightEnvironmentValid = (incoming.minHumidity != 0.0f || incoming.maxHumidity != 0.0f);
  loadFlightLogCache();
  if (flightLogCacheCount > 0 &&
      strcmp(flightLogCache[0].startTime, incoming.startTime) == 0 &&
      strcmp(flightLogCache[0].stopTime, incoming.stopTime) == 0 &&
      strcmp(flightLogCache[0].date, incoming.date) == 0) return;
  const int newCount = min(flightLogCacheCount + 1, MAX_LOGS);
  memmove(&flightLogCache[1], &flightLogCache[0], (newCount - 1) * sizeof(FlightLog));
  flightLogCache[0] = incoming; flightLogCacheCount = newCount; saveFlightLogCache();
  if (currentScreen == FLIGHT_END_SCREEN) drawFlightEndScreen();
  else if (currentScreen == FLIGHT_LOG_SCREEN) drawFlightLogScreen();
}
static void bleApplyRemoteStatus() {
  if (bleRole != BLE_ROLE_SLAVE) return;
  BleRemote r; portENTER_CRITICAL(&bleMux); r = bleRemote; portEXIT_CRITICAL(&bleMux);
  if (bleState != BLE_ST_LINKED || r.statusAtMs == 0 || millis() - r.statusAtMs > BLE_DATA_TIMEOUT_MS) return;
  const unsigned long now = millis();
  // Simulazione: lo slave segue il master solo quando lo stato del master CAMBIA (e al primo pacchetto
  // dopo il collegamento). Cosi' un play/stop appena dato sullo slave non viene annullato da un
  // pacchetto vecchio del master prima che questo abbia eseguito il comando.
  static int lastMasterDemo = -1;
  const int masterDemo = (r.statusFlags & 0x08) ? 1 : 0;
  if (masterDemo != lastMasterDemo) {
    const bool firstPacket = (lastMasterDemo < 0);
    lastMasterDemo = masterDemo;
    if (masterDemo && !demoMode) startDemoMode(now);
    else if (!masterDemo && demoMode && !firstPacket) stopDemoMode(now);
  }
  if (now - bleIctLocalAt >= 1500UL) {
    const bool masterIctEnabled = (r.statusFlags & 0x01) != 0;
    const int masterIctMinutes = constrain((int)r.remoteIctMinutes, 1, 30);
    const bool ictSettingsChanged = ictEnabled != masterIctEnabled || ictMinutes != masterIctMinutes;
    ictEnabled = masterIctEnabled;
    ictMinutes = masterIctMinutes;
    ictStartMillis = now - r.remoteIctElapsedMs;
    if (ictSettingsChanged) {
      preferences.begin("altimeter", false);
      preferences.putInt("ict_minutes", ictMinutes);
      preferences.putBool("ict_enabled", ictEnabled);
      preferences.end();
      if (currentScreen == ICT_SCREEN) drawIctScreen();
    }
  }
  const bool masterIctAlert = (r.statusFlags & 0x02) != 0;
  if (masterIctAlert && currentScreen != ICT_ALERT_SCREEN && !pageLocked) {   // bloccata: nessun avviso
    ictAlertPreviousScreen = (ScreenState)r.remoteIctPrevious;
    ictAlertStartMillis = now; ictAlarmStep = 0; ictAlarmNextMillis = 0;
    ictAlarmActive = true; ictAlertStaticDrawn = false; ictAlertTextVisible = true;
    currentScreen = ICT_ALERT_SCREEN; drawIctAlertScreen();
  } else if (!masterIctAlert && currentScreen == ICT_ALERT_SCREEN) {
    ictAlarmActive = false; currentScreen = ictAlertPreviousScreen;
    ictAlertStaticDrawn = false;
    if (!bootWizardWaiting()) redrawCurrentScreen();   // all'accensione tocca a SET QNH
  }
  const bool masterEnd = (r.statusFlags & 0x04) != 0;
  if (masterEnd && currentScreen != FLIGHT_END_SCREEN) {
    flightEndPreviousScreen = (ScreenState)r.remoteFlightEndPrevious;
    currentScreen = FLIGHT_END_SCREEN; drawFlightEndScreen();
  } else if (!masterEnd && currentScreen == FLIGHT_END_SCREEN) {
    dismissFlightEndScreen();
  }
}
void bleApplyRemoteGps() {
  if (bleRole != BLE_ROLE_SLAVE || demoMode) return;
  const unsigned long now = millis();
  BleRemote r;
  portENTER_CRITICAL(&bleMux);
  r = bleRemote;
  portEXIT_CRITICAL(&bleMux);
  const bool linked = (bleState == BLE_ST_LINKED);
  const bool sensFresh = linked && r.sensAtMs != 0 && now - r.sensAtMs <= BLE_DATA_TIMEOUT_MS;
  const bool gpsFresh = linked && r.gpsValid && r.gpsAtMs != 0 && now - r.gpsAtMs <= BLE_DATA_TIMEOUT_MS;
  const bool ownGpsOk = lastOwnFixMillis != 0 && now - lastOwnFixMillis < 2000UL;   // il GPS proprio ha la precedenza
  static bool sensWasValid = false;

  if (xSemaphoreTake(sensorDataMutex, 0) != pdTRUE) return;

  // --- sensori (lo slave non ne ha: usa sempre quelli del master) ---
  if (sensFresh) {
    const bool altOk = (r.sensFlags & 0x01) != 0;
    const bool varOk = (r.sensFlags & 0x02) != 0;
    const bool metOk = (r.sensFlags & 0x04) != 0;
    sharedFilteredAlt = r.alt;
    sharedVarioKalman = r.vario;
    sharedAltitudeValid = altOk;
    sharedVarioValid = varOk;
    rawAltitudeShared = r.alt;
    rawAltitudeValid = altOk;
    if (varOk) {
      if (r.vario > maxClimbRate) maxClimbRate = r.vario;
      if (r.vario < maxSinkRate) maxSinkRate = r.vario;
    }
    meteo_temperature_c = r.tempC;
    meteo_humidity_percent = r.hum;
    meteoValid = metOk;
    // Sonda esterna del master: AIR DATA dello slave con EXT e INT come sul master (se lo slave non ha la sua)
    const bool ownOat = oatValid && !oatFromRemote;
    if ((r.sensFlags & 0x10) && !ownOat && isfinite(r.oatC)) { oatTempC = r.oatC; oatFromRemote = true; oatValid = true; }
    else if (oatFromRemote && !(r.sensFlags & 0x10)) { oatFromRemote = false; oatValid = false; oatTempC = NAN; }
    sensWasValid = true;
  } else if (sensWasValid) {
    // collegamento perso o dati scaduti: i valori non sono piu' attendibili
    sharedAltitudeValid = false;
    sharedVarioValid = false;
    rawAltitudeValid = false;
    meteoValid = false;
    if (oatFromRemote) { oatFromRemote = false; oatValid = false; oatTempC = NAN; }
    sensWasValid = false;
  }

  // --- GPS del master, solo se il proprio non ha fix ---
  if (gpsFresh && !ownGpsOk) {
    gps_latitude = r.lat;
    gps_longitude = r.lon;
    gps_ground_speed_kmh = r.speedKmh;
    gps_course = r.course;
    gps_altitude = r.gpsAlt;
    gps_satellites = r.sats;
    gps_utc_hours = r.h; gps_utc_minutes = r.m; gps_utc_seconds = r.s;
    if (r.day > 0 && r.month > 0 && r.year >= 2000) {
      gps_utc_day = r.day; gps_utc_month = r.month; gps_utc_year = r.year;
    }

    // --- COSTELLAZIONI: applica i flag ricevuti e rinfresca i timestamp GSV ---
    gps_has_gps     = r.hasGps;
    gps_has_glonass = r.hasGlonass;
    gps_has_galileo = r.hasGalileo;
    gps_has_bds     = r.hasBds;
    gps_has_qzss    = r.hasQzss;
    if (r.hasGps)     gps_last_gsv_gps     = now;
    if (r.hasGlonass) gps_last_gsv_glonass = now;
    if (r.hasGalileo) gps_last_gsv_galileo = now;
    if (r.hasBds)     gps_last_gsv_bds     = now;
    if (r.hasQzss)    gps_last_gsv_qzss    = now;

    // --- OROLOGIO: sincronizza il clock software dal master ---
    if (r.h >= 0 && r.h <= 23 && r.m >= 0 && r.m <= 59 && r.s >= 0 && r.s <= 59 &&
        r.day > 0 && r.month > 0 && r.year >= 2000) {
      syncSoftwareClockFromGps(r.h, r.m, r.s, r.day, r.month, r.year);
    }

    if (gps_ground_speed_kmh > maxGpsSpeedKmh) maxGpsSpeedKmh = gps_ground_speed_kmh;
    gps_fix_quality = 1;
    gps_fix_type = 3;
    gpsRmcStatusValid = true;
    gpsDataValid = true;
    lastRmcMillis = now;
    lastGgaMillis = now;
    lastGpsMessageMillis = now;
  }
  xSemaphoreGive(sensorDataMutex);
}

// ========== QUOTA DENSITA' (DENSITY ALTITUDE) ==========
// Quota densita' dalla temperatura esterna corretta, dal QNH e dalla quota barometrica (stessa formula di AIR DATA).
// Conta la differenza rispetto alla quota del campo: sotto +2000 ft normale (verde), da +2000 ft arancione
// (decollo e salita piu' lunghi), da +3500 ft rosso. Corsa di decollo stimata: circa +10 % ogni 1000 ft.
// Avviso a schermo intero all'accensione (arancione o rosso; un tocco passa all'avviso successivo o alla pagina salvata);
// pagina nel giro dalla METEO: AIR DATA -> DENSITY ALTITUDE -> CARB ICE -> VISIBILITY -> SENSOR, a ogni tocco o swipe.
static const int DA_WARN_FT = 2000, DA_HIGH_FT = 3500, DA_SCALE_MAX_FT = 5000;
static bool daAlertMode = false;          // true = avviso all'accensione, false = pagina da AIR DATA
static unsigned long daShownMs = 0, daLastDrawMs = 0;
static int daLastValue = -99999, daLastLevel = -1;
static float daStartupFieldFt = NAN;      // all'accensione lo strumento e' sul campo: quota attuale = quota del campo

// Calcola quota densita' (ft), differenza sul campo (ft) e livello 0/1/2. false se i sensori non sono validi.
static bool daCompute(float &daFt, float &diffFt, int &level, float &tempC) {
  float temp = NAN, hum = NAN, alt = NAN;
  bool ok = false;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    temp = meteo_temperature_c;
    hum = meteo_humidity_percent;
    alt = sharedFilteredAlt;
    ok = sharedAltitudeValid && isfinite(alt) && meteoValid && isfinite(temp) && isfinite(hum);
    xSemaphoreGive(sensorDataMutex);
  }
  if (!ok) return false;
  float compT, compH;
  compensateOutsideAirReading(temp, hum, compT, compH);
  daFt = calculateDensityAltitude(compT, seaLevelPressure, alt);
  if (!isfinite(daFt)) return false;
  float fieldFt;
  if (daAlertMode && isfinite(daStartupFieldFt)) fieldFt = daStartupFieldFt;
  else if (qfeReferenceValid) fieldFt = qfeReferenceAltitudeMeters * 3.28084f;
  else fieldFt = alt * 3.28084f;
  diffFt = daFt - fieldFt;
  level = diffFt >= DA_HIGH_FT ? 2 : (diffFt >= DA_WARN_FT ? 1 : 0);
  tempC = compT;
  return true;
}

// Posizione sulla scala (0..1): tre settori uguali verde / arancione / rosso, allineati alle soglie
static float daScaleFrac(float diffFt) {
  if (diffFt <= 0) return 0.0f;
  if (diffFt < DA_WARN_FT) return diffFt / DA_WARN_FT / 3.0f;
  if (diffFt < DA_HIGH_FT) return 1.0f / 3.0f + (diffFt - DA_WARN_FT) / (float)(DA_HIGH_FT - DA_WARN_FT) / 3.0f;
  return min(1.0f, 2.0f / 3.0f + (diffFt - DA_HIGH_FT) / (float)(DA_SCALE_MAX_FT - DA_HIGH_FT) / 3.0f);
}

static void drawDensityAltScreenBody();
void drawDensityAltScreen() {
  const bool spr = ensureNavigationSprite();   // sprite: niente lampeggio quando cambia il valore
  if (spr) uiTarget = &navigationSprite;
  drawDensityAltScreenBody();
  uiTarget = nullptr;
  if (spr) navigationSprite.pushSprite(0, 0); drawPageLockOverlay();
}
static void drawDensityAltScreenBody() {
  ForceColorThemeScope forceColor;   // sempre a colori, come AIR DATA e gli avvisi
  statusBlinkSet(false, "", 0, 0, 0);   // riattivato sotto se la quota densita' e' alta
  const uint16_t bg = TFT_BLACK;
  UT().fillScreen(bg);
  float daFt = NAN, diffFt = 0, tempC = NAN;
  int level = 0;
  const bool ok = daCompute(daFt, diffFt, level, tempC);
  const uint16_t cGreen = themeVivid(TFT_GREEN), cOrange = TFT_ORANGE, cRed = themeVivid(TFT_RED);
  const uint16_t cLevel = level == 2 ? cRed : (level == 1 ? cOrange : cGreen);
  // Scala ad arco verde / arancione / rosso (da in basso a sinistra, passando per l'alto)
  fuelArc(UT(), 153, 167, 0.0f, 1.0f / 3.0f, cGreen);
  fuelArc(UT(), 153, 167, 1.0f / 3.0f, 2.0f / 3.0f, cOrange);
  fuelArc(UT(), 153, 167, 2.0f / 3.0f, 1.0f, cRed);
  fuelArc(UT(), 146, 147, 0.0f, 1.0f, lgfx::color565(154, 154, 154));   // arco fisso interno sottile
  if (ok) UT().fillArc(180, 180, 172, 179, 0, 360, cLevel);   // bordo sempre, del colore del livello: la pagina copre tutto il display
  drawUiVlw("DENSITY", 180, 74, MC_DATUM, TFT_CYAN, bg, 0.75f);    // titolo su due righe: non tocca l'arco
  drawUiVlw("ALTITUDE", 180, 98, MC_DATUM, TFT_CYAN, bg, 0.75f);
  if (!ok) {
    drawUiVlw("METEO INVALID", 180, 180, MC_DATUM, cRed, bg, 0.9f);
    daLastValue = -99999; daLastLevel = -1;
    return;
  }
  // Pallino bianco sull'arco nel punto del valore (al posto della lancetta)
  const float ang = (135.0f + 270.0f * daScaleFrac(diffFt)) * DEG_TO_RAD;
  const int dotX = 180 + (int)lroundf(160.0f * cosf(ang)), dotY = 180 + (int)lroundf(160.0f * sinf(ang));
  UT().fillCircle(dotX, dotY, 12, TFT_BLACK);
  UT().fillCircle(dotX, dotY, 9, TFT_WHITE);
  char buf[40];
  snprintf(buf, sizeof(buf), "%ld ft", lroundf(daFt));
  drawUiVlw(buf, 180, 140, MC_DATUM, TFT_WHITE, bg, 1.5f);   // valore piu' piccolo: spazio per la seconda riga del titolo
  snprintf(buf, sizeof(buf), "%+ld ft over field", lroundf(diffFt));
  drawUiVlw(buf, 180, 180, MC_DATUM, cLevel, bg, 0.65f);
  drawUiVlw(level == 2 ? "MUCH LONGER TAKE-OFF" : (level == 1 ? "LONGER TAKE-OFF" : "NORMAL"), 180, 212,
            MC_DATUM, cLevel, bg, level == 2 ? 0.68f : 0.8f);
  statusBlinkSet(level == 2, "MUCH LONGER TAKE-OFF", cLevel, 0.68f, 212);   // quota densita' alta: lampeggia
  UT().fillRoundRect(80, 240, 200, 62, 8, 0x0841);   // stesso riquadro di VISIBILITY e CARB ICE
  UT().drawRoundRect(80, 240, 200, 62, 8, TFT_WHITE);
  const int rollPct = diffFt > 0 ? (int)lroundf(diffFt / 1000.0f * 10.0f) : 0;
  snprintf(buf, sizeof(buf), "Take-off roll +%d %%", rollPct);
  drawUiVlw(buf, 180, 259, MC_DATUM, TFT_WHITE, 0x0841, 0.55f);
  snprintf(buf, sizeof(buf), "TEMP %.0f C - QNH %.0f", tempC, (float)seaLevelPressure);
  drawUiVlw(buf, 180, 284, MC_DATUM, TFT_WHITE, 0x0841, 0.5f);
  daLastValue = (int)lroundf(daFt);
  daLastLevel = level;
  daLastDrawMs = millis();
}


// ========== ONLINE WEATHER: il meteo dal telefono a caratteri grandi ==========
// Primo passo del giro che parte con un tocco su AIR DATA. Fuori dal giro con lo swipe.
// In alto si scrive sempre DI CHE ZONA sono i numeri: senza quello una visibilita' di 8 km
// non dice niente, perche' non sai se e' la tua o quella del campo a settanta chilometri.
// Le sei icone della pagina ONLINE WEATHER, disegnate con cerchi, triangoli e rettangoli: niente
// immagini da caricare e niente byte in piu' in memoria. Riquadro di 26 px centrato in cx, cy.
// Sul tema non a colori diventano nere come tutto il resto, altrimenti sparirebbero sul chiaro.
static void drawWxIcon(int kind, int cx, int cy, uint16_t col) {
  const uint16_t c = (displayTheme == THEME_COLOR) ? col : TFT_BLACK;
  const uint16_t bg = themeBg();
  switch (kind) {
    case 0:   // QNH: cerchi concentrici
      UT().drawCircle(cx, cy, 11, c);
      UT().drawCircle(cx, cy, 10, c);
      UT().drawCircle(cx, cy, 6, c);
      UT().fillCircle(cx, cy, 2, c);
      break;
    case 1:   // WIND: punta piena, con l'incavo dietro
      UT().fillTriangle(cx - 9, cy - 10, cx + 11, cy, cx - 9, cy + 10, c);
      UT().fillTriangle(cx - 9, cy - 10, cx - 3, cy, cx - 9, cy + 10, bg);
      break;
    case 2:   // GUST: soffio, tre strisce con il ricciolo
      UT().fillRoundRect(cx - 12, cy - 8, 14, 3, 1, c);
      UT().fillRoundRect(cx + 1, cy - 11, 3, 4, 1, c);
      UT().fillRoundRect(cx - 12, cy - 2, 19, 3, 1, c);
      UT().fillRoundRect(cx - 10, cy + 4, 12, 3, 1, c);
      UT().fillRoundRect(cx + 1, cy + 5, 3, 4, 1, c);
      break;
    case 3:   // VIS: occhio
      UT().drawEllipse(cx, cy, 12, 7, c);
      UT().drawEllipse(cx, cy, 11, 6, c);
      UT().fillCircle(cx, cy, 3, c);
      break;
    case 4:   // FRZ: termometro con due tacche
      UT().drawRoundRect(cx - 5, cy - 12, 9, 16, 4, c);
      UT().drawRoundRect(cx - 4, cy - 11, 7, 14, 3, c);
      UT().fillCircle(cx - 1, cy + 6, 5, c);
      UT().fillRect(cx + 6, cy - 8, 5, 2, c);
      UT().fillRect(cx + 6, cy - 3, 5, 2, c);
      break;
    case 5:   // RAIN: nuvola piena con tre gocce
      UT().fillCircle(cx - 5, cy - 4, 5, c);
      UT().fillCircle(cx + 3, cy - 6, 6, c);
      UT().fillRect(cx - 10, cy - 4, 17, 5, c);
      UT().fillRect(cx - 7, cy + 3, 2, 5, c);
      UT().fillRect(cx - 1, cy + 3, 2, 5, c);
      UT().fillRect(cx + 5, cy + 3, 2, 5, c);
      break;
  }
}

void drawOnlineWxScreen() {
  const bool spr = ensureNavigationSprite();
  if (spr) uiTarget = &navigationSprite;
  const uint16_t bg = themeBg();
  if (spr) navigationSprite.fillScreen(bg); else lcd.fillScreen(bg);

  // Il titolo "ONLINE WEATHER" non si scrive piu': mangiava 28 px di altezza senza dire niente
  // che il nome del campo non dica meglio, e quei 28 px servono a far crescere i numeri.
  // Zona: il campo del GO TO, altrimenti il campo di casa, altrimenti la posizione attuale.
  char zona[44];
  const int fi = navGotoActive ? navGotoIdx : frHomeFieldIdx();
  if (navGotoActive && fi >= 0 && fi < AIRFIELD_COUNT)
    snprintf(zona, sizeof(zona), "%.20s", AIRFIELDS[fi].name);
  else
    snprintf(zona, sizeof(zona), "CURRENT POSITION");
  // Il nome del campo scende dove stava la riga della destinazione, piu' 5 px: a 24 le lettere
  // toccavano il bordo del vetro, perche' lassu' il tondo e' larga meno di 140 px.
  drawUiVlw(zona, 180, 48, MC_DATUM, themeColor(TFT_WHITE), bg, 0.60f);

  // Campo appena cambiato: i numeri che abbiamo sono dell'altro posto, quindi non si
  // scrivono. Dieci o venti secondi e il telefono manda quelli giusti.
  const bool allineato = metAligned();
  char sub[40];
  const int age = metAgeMin();
  if (metFresh() && !allineato)
    snprintf(sub, sizeof(sub), "UPDATING");
  else if (metFresh() && age >= 0)
    snprintf(sub, sizeof(sub), "%s - %d min", navGotoActive ? "DESTINATION" : "HERE", age);
  else
    snprintf(sub, sizeof(sub), "NO DATA FROM PHONE");
  // La riga della destinazione va in fondo alla pagina: a 324, cioe' sotto l'ultima riga di
  // valori e dentro all'archetto del conto alla rovescia, che a quell'altezza passa a 83 e 277.
  drawUiVlw(sub, 180, 324, MC_DATUM,
            (metFresh() && allineato) ? themeVivid(TFT_GREEN) : themeColor(TFT_DARKGREY), bg, 0.40f);

  if (!metFresh()) {
    drawUiVlw("Open the AVEA Link app", 180, 180, MC_DATUM, themeColor(TFT_DARKGREY), bg, 0.44f);
    drawUiVlw("on the phone", 180, 206, MC_DATUM, themeColor(TFT_DARKGREY), bg, 0.44f);
    uiTarget = nullptr;
    if (spr) navigationSprite.pushSprite(0, 0);
    drawPageLockOverlay();
    return;
  }

  // Sei righe in colonna, sempre tutte e sei, con l'icona davanti all'etichetta. Le posizioni
  // sono fisse e non si adattano: icona centrata in 78, etichetta allineata a destra a 142,
  // valore allineato a sinistra a 154, unita' piccola subito dopo il valore. Cosi' i numeri
  // stanno incolonnati e si leggono scorrendo l'occhio in verticale, invece di spostarsi a
  // ogni riga come farebbero se ogni riga fosse centrata.
  // L'unita' di misura e' staccata dal valore e piccola: attaccata al numero grande avrebbe
  // allungato la riga del vento oltre il bordo del vetro.
  // Altezze scelte sulla geometria del tondo: il raggio e' 180, quindi a quota dy dal centro
  // la mezza larghezza vale radice(180^2 - dy^2). La riga piu' lunga e' il vento, che sta in
  // alto dove c'e' spazio; le righe in basso, dove il display si restringe, hanno i valori
  // corti. Prima riga a 76 e passo 43: l'ultima cade a 291 e ci sta dentro con il margine.
  const float SCV = 1.28f, SCE = 0.53f, SCU = 0.40f;
  const int ROW0 = 76, STEP = 43;
  const int ICON_X = 78, ET_X = 142, VA_X = 154;
  static const uint16_t WX_ICON_COL[6] = { TFT_CYAN, TFT_GREEN, TFT_ORANGE, TFT_WHITE, TFT_CYAN, TFT_BLUE };
  char et[10], va[24], un[8];
  for (int k = 0; k < 6; ++k) {
    et[0] = 0; va[0] = 0; un[0] = 0;
    uint16_t col = themeColor(TFT_WHITE);
    switch (k) {
      case 0: snprintf(et, sizeof(et), "QNH");
              if (metQnhValid) {
                snprintf(va, sizeof(va), "%d", (int)lroundf((float)metQnh));
                snprintf(un, sizeof(un), "hPa");
              }
              break;
      case 1: snprintf(et, sizeof(et), "WIND");
              if (metWindValid) {
                snprintf(va, sizeof(va), "%03d  %d", metWindDir % 360, (int)lroundf((float)metWindKt));
                snprintf(un, sizeof(un), "kt");
                if (metWindStrong()) col = themeVivid(TFT_RED);
              }
              break;
      case 2: snprintf(et, sizeof(et), "GUST");
              if (metWindValid && metGustKt >= metWindKt + 3.0f) {
                snprintf(va, sizeof(va), "%d", (int)lroundf((float)metGustKt));
                snprintf(un, sizeof(un), "kt");
                // Ambra solo se la raffica e' significativa secondo la soglia comune:
                // vento >= 15 kt oppure raffica almeno 8 kt sopra il vento.
                if (metWindStrong()) col = themeVivid(TFT_ORANGE);
              }
              break;
      case 3: snprintf(et, sizeof(et), "VIS");
              if (metVisValid) {
                snprintf(va, sizeof(va), "%d", (int)lroundf(metVisM / 1000.0f));
                snprintf(un, sizeof(un), "km");
              }
              break;
      case 4: snprintf(et, sizeof(et), "FRZ");
              if (metFrzValid) {
                snprintf(va, sizeof(va), "%d", metFrzM);
                snprintf(un, sizeof(un), "m");
              }
              break;
      case 5: snprintf(et, sizeof(et), "RAIN");
              if (metPrecipValid) {
                snprintf(va, sizeof(va), "%d", metPrecipProb);
                snprintf(un, sizeof(un), "%%");
              }
              break;
    }
    if (!allineato) { va[0] = 0; un[0] = 0; }   // meteo dell'altro campo: tutte le righe a trattino
    if (va[0] == 0) { snprintf(va, sizeof(va), "--"); un[0] = 0; col = themeColor(TFT_DARKGREY); }
    const int y = ROW0 + k * STEP;
    drawWxIcon(k, ICON_X, y, themeVivid(WX_ICON_COL[k]));
    drawUiVlw(et, ET_X, y, MR_DATUM, themeColor(TFT_WHITE), bg, SCE);
    const int wv = drawUiVlw(va, VA_X, y, ML_DATUM, col, bg, SCV);
    if (un[0]) drawUiVlw(un, VA_X + wv + 7, y, ML_DATUM, themeColor(TFT_DARKGREY), bg, SCU);
  }

  // Pagina uscita da sola: un archetto in basso sulla corona dice quanto le resta. Si
  // consuma da destra verso sinistra e quando e' finito la pagina se ne va. Un tocco
  // qualsiasi la manda via prima.
  if (wxAutoUntilMs != 0) {
    const long rest = (long)(wxAutoUntilMs - millis());
    float f = rest <= 0 ? 0.0f : (float)rest / (float)MET_WX_AUTO_MS;
    if (f > 1.0f) f = 1.0f;
    const float A0 = 40.0f, A1 = 140.0f;   // gradi, 0 = ore 3, crescendo verso il basso
    for (float a = A0; a <= A1 + 0.01f; a += 2.5f) {
      const bool acceso = (a - A0) <= (A1 - A0) * f;
      const float rr = a * DEG_TO_RAD;
      const int px = (int)lroundf(180.0f + 174.0f * cosf(rr));
      const int py = (int)lroundf(180.0f + 174.0f * sinf(rr));
      UT().fillCircle(px, py, 2, acceso ? themeColor(TFT_CYAN) : themeColor(TFT_DARKGREY));
    }
  }

  uiTarget = nullptr;
  if (spr) navigationSprite.pushSprite(0, 0);
  drawPageLockOverlay();
}

void openOnlineWxPage() {
  currentScreen = ONLINE_WX_SCREEN;
  drawOnlineWxScreen();
}

void openDensityAltPage() {
  daAlertMode = false;
  currentScreen = DENSITY_ALT_SCREEN;
  daShownMs = millis();
  drawDensityAltScreen();
}

// All'accensione: true se l'avviso e' stato aperto (arancione o rosso)
bool densityAltStartupCheck() {
  float alt = NAN;
  if (sensorDataMutex != nullptr && xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
    if (sharedAltitudeValid && isfinite(sharedFilteredAlt)) alt = sharedFilteredAlt;
    xSemaphoreGive(sensorDataMutex);
  }
  if (!isfinite(alt)) return false;
  daStartupFieldFt = alt * 3.28084f;
  daAlertMode = true;
  float daFt, diffFt, tempC;
  int level = 0;
  if (!daCompute(daFt, diffFt, level, tempC) || level == 0) { daAlertMode = false; return false; }
  AVEA_LOGF("DENSITY ALT: %.0f ft, %+.0f ft sul campo, livello %d\n", daFt, diffFt, level);
  if (isTouching) { ignoreTouchUntilRelease = true; isTouching = false; }
  currentScreen = DENSITY_ALT_SCREEN;
  daShownMs = millis();
  drawDensityAltScreen();
  playTone(1200, 120); playTone(1600, 120);
  return true;
}

void finishStartupContinue();
void bootWizardArm();
bool bootWizardWaiting();
// Tocco o swipe: l'avviso di avvio continua la sequenza (rischi, poi pagina salvata); la pagina torna a SENSOR
void dismissDensityAlt() {
  if (currentScreen != DENSITY_ALT_SCREEN) return;
  lastTouchMillis = millis();
  if (daAlertMode) {
    daAlertMode = false;
    finishStartupContinue();
    return;
  }
  currentScreen = METEO_SCREEN;
  for (int i = 0; i < numScreens; ++i) { if (screenOrder[i] == METEO_SCREEN) { currentScreenIndex = i; break; } }
  redrawCurrentScreen();
}

// ---- Avvisi letti su master e slave insieme ----
// Avviso mostrato ora: 1 = DENSITY ALTITUDE, 2 = CARB ICE, 3 = VISIBILITY, 0 = nessuno (o giro di prova dalla SENSOR).
static uint8_t alertKindOnScreen() {
  if (riskPreviewActive || demoRiskTourActive) return 0;
  if (currentScreen == DENSITY_ALT_SCREEN) return 1;
  if (currentScreen == RISK_WARNING_SCREEN) {
    const bool showCarb = riskWarningForcedPage == 1 ? true
                        : (riskWarningForcedPage == 2 ? false : riskWarningCarbLevel >= riskWarningFogLevel);
    return showCarb ? 2 : 3;
  }
  return 0;
}
// Stesso effetto del tocco sull'avviso: chiude quello mostrato (all'accensione passa al successivo o alla pagina).
static void alertTapLocal() {
  if (currentScreen == DENSITY_ALT_SCREEN) { dismissDensityAlt(); return; }
  if (currentScreen != RISK_WARNING_SCREEN) return;
  if (riskWarningStepNext != 0) {
    riskWarningForcedPage = riskWarningStepNext;   // accensione: avviso successivo
    riskWarningStepNext = 0;
    drawRiskWarningScreen();
  } else {
    dismissRiskWarningScreen();
  }
}
static uint8_t alertAckLastLocalKind = 0;          // ultimo avviso letto qui (per ignorarne l'eco dal master)
static unsigned long alertAckLastLocalMs = 0;
static uint8_t alertAckPendingKind = 0;            // avviso letto sull'altro strumento, non ancora mostrato qui
static unsigned long alertAckPendingMs = 0;
// Tocco su un avviso: lo segnala all'altro strumento
static void alertAckSend(uint8_t kind) {
  if (kind == 0 || bleRole == BLE_ROLE_OFF) return;
  alertAckLastLocalKind = kind; alertAckLastLocalMs = millis();
  portENTER_CRITICAL(&bleMux);
  if (bleRole == BLE_ROLE_SLAVE) { bleAckTxKind = kind; bleAckTxAt = millis(); }
  else { bleAckSeq = (uint8_t)(bleAckSeq + 1); bleAckKind = kind; bleAckSendUntil = millis() + 20000UL; }
  portEXIT_CRITICAL(&bleMux);
}
// Avviso letto sull'altro strumento: se e' quello mostrato si chiude, altrimenti si aspetta 15 s che compaia.
static void alertAckApply(uint8_t kind) {
  if (kind == 0) return;
  if (alertKindOnScreen() == kind) { alertTapLocal(); playTone(1200, 50); alertAckPendingKind = 0; return; }
  alertAckPendingKind = kind; alertAckPendingMs = millis();
}
// Nel loop (master e slave)
void alertAckLoop() {
  if (bleRole == BLE_ROLE_MASTER && bleAckRxKind != 0) {   // toccato sullo slave
    portENTER_CRITICAL(&bleMux);
    const uint8_t k = bleAckRxKind; bleAckRxKind = 0;
    bleAckSeq = (uint8_t)(bleAckSeq + 1); bleAckKind = k; bleAckSendUntil = millis() + 20000UL;   // anche agli altri slave
    portEXIT_CRITICAL(&bleMux);
    alertAckApply(k);
  }
  if (bleRole == BLE_ROLE_SLAVE && bleAckRemoteNew) {      // toccato sul master (o su un altro slave)
    portENTER_CRITICAL(&bleMux);
    const uint8_t k = bleAckRemoteKind; bleAckRemoteNew = false;
    portEXIT_CRITICAL(&bleMux);
    const bool echo = k == alertAckLastLocalKind && millis() - alertAckLastLocalMs < 25000UL;   // l'ho toccato io
    if (!echo) alertAckApply(k);
  }
  if (alertAckPendingKind != 0) {
    if (millis() - alertAckPendingMs > 15000UL) alertAckPendingKind = 0;
    else if (alertKindOnScreen() == alertAckPendingKind) { alertAckPendingKind = 0; alertTapLocal(); playTone(1200, 50); }
  }
}

// Nel loop: aggiorna i valori ogni 2 s (l'avviso di avvio resta fino al tocco)
void updateDensityAltScreen() {
  if (currentScreen != DENSITY_ALT_SCREEN) return;
  const unsigned long now = millis();
  if (now - daLastDrawMs < 2000UL) return;
  float daFt, diffFt, tempC;
  int level = 0;
  if (!daCompute(daFt, diffFt, level, tempC)) { if (daLastLevel != -1) drawDensityAltScreen(); return; }
  if ((int)lroundf(daFt) / 10 != daLastValue / 10 || level != daLastLevel) drawDensityAltScreen();
  else daLastDrawMs = now;
}

// Fine del logo di avvio: mostra l'eventuale avviso ghiaccio/nebbia (che al tocco
// torna alla pagina salvata) oppure direttamente la pagina salvata.
void finishStartupLogo() {
  if (!startupSequenceActive) return;
  startupSequenceActive = false;
  if (!licenseValid) {
    // Strumento non attivato: prima la pagina di attivazione; uscendo (swipe) parte la DEMO
    startupRiskCheckPending = false;
    lastTouchMillis = millis();
    openLicenseScreen(startupTargetScreen == WIFI_LOGS_SCREEN ? (int)FIRMWARE_SCREEN : (int)startupTargetScreen);
    return;
  }
  if (densityAltStartupCheck()) return;   // quota densita' alta: prima l'avviso, poi finishStartupContinue()
  finishStartupContinue();
}

// Seconda parte dell'avvio: pagina salvata ed eventuale avviso ghiaccio/nebbia
void finishStartupContinue() {
  currentScreen = startupTargetScreen;
  for (int i = 0; i < numScreens; ++i) {
    if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
  }
  if (currentScreen == WIFI_LOGS_SCREEN) {
    wifiReturnScreen = screenOrder[nextEnabledScreenIndex(currentScreenIndex, -1)];
    startLogWifi();
  }
  lastTouchMillis = millis();
  startupRiskCheckPending = true;
  startupRiskCheckDeadlineMillis = millis() + startupRiskWaitMs();
  bootWizardArm();   // dopo gli avvisi: SET QNH, SET LITERS, poi questa pagina
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "fine logo", millis());
  checkRuntimeRiskWarning();   // con rischio MEDIUM/HIGH apre subito l'avviso (previous = pagina salvata)
  // Con la sequenza SET QNH in arrivo resta il logo (niente lampo della pagina salvata)
  if (currentScreen != RISK_WARNING_SCREEN && !bootWizardWaiting()) redrawCurrentScreen();
}

// ========== ACCENSIONE: SET QNH -> SET LITERS -> pagina salvata ==========
// Dopo gli avvisi di avvio (DENSITY ALT, CARB ICE, VISIBILITY...) si apre SET QNH con il pulsante NEXT;
// poi SET LITERS (solo il rullo dei litri, aggiorna FUEL ESTIMATE) se la pagina FUEL ESTIMATE e' attiva
// in PAGES; all'uscita la pagina salvata nelle preferenze. Swipe dx/sx = salta senza salvare.
// 10 s senza tocchi = va avanti da solo: gli avvisi passano al successivo, SET QNH tiene il valore
// mostrato (o prende quello del telefono se c'e'), SET LITERS salta senza salvare. Toccando la pagina
// il conto riparte, cosi' si legge con calma. Niente sequenza in volo reale, in DEMO o con la pagina
// WIFI LOG all'avvio.
static bool bootWizPending = false;   // da aprire appena finiti gli avvisi di avvio
static bool bootWizActive = false;    // SET QNH o SET LITERS di accensione a schermo
static unsigned long bootWizArmMs = 0, bootWizIdleMs = 0;
static unsigned long bootWizAlertMs = 0;   // da quando questo avviso e' a schermo
static int bootWizAlertKey = -1;           // quale avviso: cambia anche fra ghiaccio e nebbia
const unsigned long BOOT_WIZ_IDLE_MS = 10000UL;     // senza tocchi: avanti da solo
const unsigned long BOOT_WIZ_GIVEUP_MS = 120000UL;  // avvisi aperti troppo a lungo: niente sequenza

bool bootLitersWanted() {
  if (realFlightActive()) return false;
  for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == FUEL_SCREEN) return pageIndexEnabled(i);
  return false;
}
static bool bootWizAlertOnScreen() {
  return currentScreen == RISK_WARNING_SCREEN || currentScreen == DENSITY_ALT_SCREEN ||
         currentScreen == ICT_ALERT_SCREEN || currentScreen == AIRSPACE_ALERT_SCREEN ||
         currentScreen == FUEL_RESERVE_SCREEN || currentScreen == SUNSET_SCREEN ||
         currentScreen == LICENSE_SCREEN || currentScreen == AVEA_LOGO_SCREEN ||
         currentScreen == FLIGHT_END_SCREEN || currentScreen == WIFI_LOGS_SCREEN ||
         currentScreen == WIFI_LOGIN_SCREEN || currentScreen == FIRMWARE_SCREEN;
}
void bootWizardArm() {
  bootWizActive = false;
  bootWizPending = startupTargetScreen != WIFI_LOGS_SCREEN && !demoMode && !realFlightActive();
  bootWizArmMs = millis();
}
bool bootWizardWaiting() { return bootWizPending; }

static void bootWizOpenQnh() {
  if (isTouching) { ignoreTouchUntilRelease = true; isTouching = false; }   // il dito gia' appoggiato non conta
  qnhBootMode = true;
  qnhBootStartHpa = seaLevelPressure;
  qnhPendingMeters = altitudeUnitsIsMeters;
  bootWizActive = true;
  bootWizIdleMs = millis();
  currentScreen = QNH_SETUP_SCREEN;
  drawQnhSetupScreen();
  playTone(1200, 80);
}

// SET QNH (BACK / SAVE della pagina): salva QNH e METERS / FEET
void qnhSetupSave() {
  altitudeUnitsIsMeters = qnhPendingMeters;
  preferences.begin("altimeter", false);
  preferences.putFloat("qnh", seaLevelPressure);
  preferences.putBool("is_meters", altitudeUnitsIsMeters);
  preferences.end();
  qnhChangedAtMillis = millis();
  if (bleRole == BLE_ROLE_SLAVE) bleQueueQnh(seaLevelPressure);   // il master lo applica e lo rinvia
  bleQueuePrefs();   // slave: METERS / FEET anche sul master
}

// Dopo SET QNH di accensione: SET LITERS oppure direttamente la pagina salvata
void bootWizardAfterQnh() {
  qnhBootMode = false;
  if (bootLitersWanted()) {
    fsBootMode = true;
    openFuelSetupScreen();
    if (currentScreen == FUEL_SETUP_SCREEN) { bootWizIdleMs = millis(); return; }
    fsBootMode = false;
  }
  bootWizardFinish();
}

// Swipe su SET QNH di accensione: salta senza salvare (torna il QNH di prima)
void bootWizardSkipQnh() {
  if (seaLevelPressure != qnhBootStartHpa) {
    seaLevelPressure = qnhBootStartHpa;
    if (bleRole == BLE_ROLE_SLAVE) bleQueueQnh(seaLevelPressure);
  }
  bootWizardAfterQnh();
}

void bootWizardFinish() {
  bootWizActive = false;
  bootWizPending = false;
  qnhBootMode = false;
  fsBootMode = false;
  currentScreen = startupTargetScreen;
  for (int i = 0; i < numScreens; ++i) if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
  lastTouchMillis = millis();
  redrawCurrentScreen();
}

// Nel loop
void bootWizardLoop() {
  const unsigned long now = millis();
  if (bootWizPending) {
    if (startupSequenceActive) return;
    if (startupRiskCheckPending && (long)(now - startupRiskCheckDeadlineMillis) < 0) return;   // controllo avvisi (max 3 s)
    if (realFlightActive() || demoMode || now - bootWizArmMs > BOOT_WIZ_GIVEUP_MS) {
      bootWizPending = false;
      if (!bootWizAlertOnScreen()) redrawCurrentScreen();   // c'era ancora il logo
      return;
    }
    // Avvisi che avanzano da soli: dieci secondi a pagina e poi al successivo, come se li avessi
    // toccati, cosi' l'accensione arriva in fondo senza che tu tocchi niente. Il conto riparte
    // mentre il dito e' appoggiato, per leggere con calma. Solo l'avviso di quota densita' e
    // quelli di ghiaccio/nebbia: la pagina di attivazione no, perche' uscire da quella avvia la
    // DEMO e non e' una cosa che debba succedere da sola.
    if (currentScreen == RISK_WARNING_SCREEN || currentScreen == DENSITY_ALT_SCREEN) {
      const int key = (int)currentScreen * 16 + (int)riskWarningForcedPage;
      if (key != bootWizAlertKey) { bootWizAlertKey = key; bootWizAlertMs = now; }
      if (isTouching) bootWizAlertMs = now;
      if (now - bootWizAlertMs >= BOOT_WIZ_IDLE_MS) {
        bootWizAlertMs = now;
        alertTapLocal();
        playTone(1200, 50);
      }
      return;
    }
    if (bootWizAlertOnScreen()) return;   // aspetta la chiusura dell'avviso
    bootWizPending = false;
    bootWizOpenQnh();
    return;
  }
  if (!bootWizActive) return;
  if (currentScreen == QNH_SETUP_SCREEN && qnhBootMode) {
    if (isTouching) bootWizIdleMs = now;
    if (now - bootWizIdleMs >= BOOT_WIZ_IDLE_MS) {
      // Allo scadere si prende il QNH del telefono, se e' arrivato ed e' ancora fresco: e' un dato
      // misurato sulla zona, meglio di quello che era in memoria dall'ultimo volo. Se non c'e',
      // resta quello mostrato. In entrambi i casi si salva, come farebbe il tasto SAVE.
      if (metQnhValid && metFresh()) seaLevelPressure = (float)metQnh;
      qnhSetupSave(); bootWizardAfterQnh(); playTone(1500, 80);
    }
  } else if (currentScreen == FUEL_SETUP_SCREEN && fsBootMode) {
    if (isTouching || fsFlingCol >= 0) bootWizIdleMs = now;
    // Allo scadere i litri NON si salvano: nessuno li ha confermati, e scrivere un livello di
    // carburante che nessuno ha guardato e' peggio che lasciare quello di prima.
    if (now - bootWizIdleMs >= BOOT_WIZ_IDLE_MS) fuelSetupExit();
  } else if (!bootWizAlertOnScreen()) {
    bootWizActive = false;   // uscita in altro modo (es. volo reale): sequenza finita
  }
}

void setup() {
  oatInit();   // sonda esterna DS18B20 su GPIO13
  preferences.begin("altimeter", false);
  const int nvsVersion = preferences.getInt("nvs_ver", 0);
  if (nvsVersion < 1) {
    preferences.clear();
    preferences.putInt("nvs_ver", 1);
    AVEA_LOGLN("[NVS] namespace altimeter azzerato (prima inizializzazione)");
  } else {
    AVEA_LOGLN("[NVS] namespace altimeter preservato");
  }
  preferences.end();

  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);  // se nessuno legge la seriale USB (Arduino chiuso) scarta il testo invece di aspettare 100 ms a ogni stampa
#endif
  delay(50);
  AVEA_LOGF("PSRAM: %s, dimensione %u byte, heap interno libero %u byte\n", psramFound() ? "TROVATA" : "NON trovata (Strumenti > PSRAM > OPI PSRAM)", (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap());
  qfeResetAtMillis = millis() + 13000UL;
  // [AVVIO IMMEDIATO] Rimossa l'attesa fissa di 500 ms: serviva solo a dare
  // tempo al monitor seriale di collegarsi prima del primo log di debug,
  // non e' necessaria per il funzionamento dello strumento.
  AVEA_LOGF("Motivo reset ESP32: %d\n", (int)esp_reset_reason());
  AVEA_LOGLN("Inizializzazione watchdog");
  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&wdtConfig);
  powerOnAndResetHardware();
  // La SD viene montata solo quando si richiede Update from SD: evita di
  // inizializzare SDMMC durante l'avvio del touch e del display.
  initAudioI2S();   // codec audio: l'amplificatore si accende solo con AUDIO ON in PAGES 2/2
  GpsSerial.setRxBufferSize(2048);  // va chiamato PRIMA di begin(); il default (256 byte) trabocca in fretta a 10 Hz
  GpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  const unsigned long gpsSerialStartedAt = millis();
  lcd.init();
  loadVerticalSwipeSetting();   // legge subito ROTATE DISPLAY 180: anche il logo di avvio esce gia' ruotato
  applyDisplayRotation();
  initTouchI2cBus();
  configureCST816NoSleep();
  if (touchI2cReady) {
    touchIntFlag = true;  // forza una prima lettura
    attachInterrupt(digitalPinToInterrupt(TOUCH_INT_PIN), onTouchInt, FALLING);
    touchIntAttached = true;
  }
  lcd.setBrightness(255);
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "display", millis());
  lcd.fillScreen(themeBg());


  // BME280: piu' tentativi all'avvio (sensore esterno su cavo, alimentazione che si assesta, bus appena
  // avviato). Prima un solo tentativo fallito bastava a considerare lo strumento "senza sensori" fino al
  // riavvio successivo.
  bmeReady = false;
  for (int attempt = 0; attempt < 6 && !bmeReady; ++attempt) {
    if (attempt > 0) { esp_task_wdt_reset(); delay(150UL * attempt); }
    bmeReady = initBME280();
    if (!bmeReady) AVEA_LOGF("BME280: tentativo %d all'avvio fallito\n", attempt + 1);
  }
  bmeFoundAtBoot = bmeReady;
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "bme280", millis());
  {
#if AVEA_SERIAL_LOG
    // Scansione dei 112 indirizzi del bus I2C: serve SOLO alla riga di log qui sotto, per
    // diagnosticare i collegamenti. Col log spento la riga sparisce ma la scansione girava
    // comunque a ogni accensione, un centinaio di transazioni per niente: ora va col log.
    String found;
    for (uint8_t a = 0x08; a < 0x78; ++a) {
      Wire.beginTransmission(a);
      if (Wire.endTransmission() == 0) { char b[8]; snprintf(b, sizeof(b), " 0x%02X", a); found += b; }
    }
    AVEA_LOGF("I2C trovati:%s\n", found.c_str());
#endif
    const bool ok = slipBall.begin(Wire);
    AVEA_LOGF("TURN & SLIP: %s (ID letto 0x%02X)\n", ok ? "MPU-6050 OK (giroscopio)" : "sensore non trovato", slipBall.lastDevId);
  }
  if (!bmeReady) {
    AVEA_LOGLN("BME280 non trovato all'avvio: nessun recupero automatico (strumento senza sensori, es. slave BLE). Riavviare se il sensore e' montato.");
  } else {
    AVEA_LOGLN("BME280 inizializzato.");
  }

  preferences.begin("altimeter", false);
  seaLevelPressure = constrain(preferences.getFloat("qnh", 1013.25f), QNH_MIN_HPA, QNH_MAX_HPA);
  altitudeUnitsIsMeters = preferences.getBool("is_meters", true);
  // audio: ora si sceglie in PAGES 2/2 (NVS "pages"/"audio", di serie spento), letto con le altre impostazioni di PAGES
  isLightMode = preferences.getBool("av_light", true);
  buzzEnabled = preferences.getBool("av_buzz", true);
  waveMode = preferences.getBool("av_wave", false);
  avThresholdIndex = preferences.getInt("av_threshold", 3);
  if (avThresholdIndex < 0 || avThresholdIndex > 3) avThresholdIndex = 3;
  varioUpThreshold = avLiftThresholds[avThresholdIndex];
  varioDownThreshold = -varioUpThreshold;
  ictMinutes = preferences.getInt("ict_minutes", 15);
  if (ictMinutes < 1 || ictMinutes > 30) ictMinutes = 15;
  ictEnabled = preferences.getBool("ict_enabled", false);
  ictStartMillis = millis();
  timezoneIndex = preferences.getInt("tz_idx", 0);
  if (timezoneIndex < 0 || timezoneIndex > 4) timezoneIndex = 0;
  {   // fuso automatico: ultima posizione nota finche' non arriva il GPS
    const double la = preferences.getDouble("tz_lat", 45.4642), lo = preferences.getDouble("tz_lon", 9.19);
    if (isfinite(la) && isfinite(lo) && fabs(la) <= 90.0 && fabs(lo) <= 180.0) { tzAutoLat = la; tzAutoLon = lo; }
    tzAutoRule = tzaRuleAt(tzAutoLat, tzAutoLon);
  }
  audioVolumePercent = preferences.getInt("av_volume", 70);
  if (audioVolumePercent < 0 || audioVolumePercent > 100) audioVolumePercent = 70;
  // Pagina FILTER nascosta: filtro vario sempre NORMAL (ignora un eventuale valore salvato prima).
  setVarioFilterLevel(FILTER_NORMAL);
  if (preferences.getInt("vario_filter", FILTER_NORMAL) != FILTER_NORMAL) preferences.putInt("vario_filter", FILTER_NORMAL);
  lubberWidthIdx = preferences.getInt("lub_w", 0);
  if (lubberWidthIdx < 0 || lubberWidthIdx > 3) lubberWidthIdx = 0;
  lubberYellowIdx = preferences.getInt("lub_y", 0);
  if (lubberYellowIdx < 0 || lubberYellowIdx > 2) lubberYellowIdx = 0;
  int audioProfileVersion = preferences.getInt("av_profile_ver", 0);
  if (audioProfileVersion < 2) {
    avThresholdIndex = 3;
    waveMode = false;
    varioUpThreshold = avLiftThresholds[3];
    varioDownThreshold = -varioUpThreshold;
    preferences.putInt("av_threshold", avThresholdIndex);
    preferences.putBool("av_wave", waveMode);
    preferences.putInt("av_profile_ver", 2);
  }
  // Migrazione una tantum: soglia di attivazione vario a 1.0 m/s (indice 3).
  if (preferences.getInt("vario_thr_ver", 0) < 1) {
    avThresholdIndex = 3;
    varioUpThreshold = avLiftThresholds[3];
    varioDownThreshold = -varioUpThreshold;
    preferences.putInt("av_threshold", avThresholdIndex);
    preferences.putInt("vario_thr_ver", 1);
  }
  // Pagina SETUP nascosta: soglia di attivazione vario sempre 1.0 m/s (indice 3).
  avThresholdIndex = 3;
  varioUpThreshold = avLiftThresholds[3];
  varioDownThreshold = -varioUpThreshold;
  if (preferences.getInt("av_threshold", 3) != 3) preferences.putInt("av_threshold", 3);
  AVEA_LOGF("Vario threshold: indice %d = %.1f m/s\n", avThresholdIndex, avLiftThresholds[avThresholdIndex]);
  // Di serie MASTER, non OFF: lo strumento che si vende e' un master, e con OFF il servizio
  // Bluetooth non viene nemmeno creato, quindi l'app AVEA Link resta in cerca per sempre senza
  // che niente, ne' sul telefono ne' sul display, dica il perche'. Lo SLAVE e' sempre una scelta
  // deliberata di chi ha un secondo apparecchio, e gli costa un tocco in GPS INFO. Non si decide
  // da soli in base ai sensori: se per un contatto incerto il BME280 non venisse trovato
  // all'accensione, uno strumento completo si metterebbe in slave e resterebbe ad aspettare un
  // master che non esiste, senza GPS, senza traccia e senza log.
  bleRole = preferences.getUChar("ble_role", BLE_ROLE_MASTER);
  if (bleRole > BLE_ROLE_SLAVE) bleRole = BLE_ROLE_OFF;
  preferences.end();
  loadFlightLogCache();  // carica (ed eventualmente migra) i log voli in RAM
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "log voli", millis());
  trackMount();          // memoria delle tracce GPS (LittleFS)
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "littlefs", millis());
  loadAirspaceSettings();
  loadFieldsRadarZoom();   // zoom della pagina FIELDS (10 / 20 / 30 km)
  airspaceLoad();        // spazi aerei (file caricato dalla pagina web) nella PSRAM
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "spazi aerei", millis());
  waypointsLoad();       // elenco dei campi: file WAYPOINTS se caricato, altrimenti quello del firmware
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "campi", millis());
  sweepLutInit();        // tabelle della scia delle lancette (GPS SPEED, ALTIMETER, QFE)
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "tabelle", millis());
  // La radio GPS richiede 300 ms dopo l'apertura della seriale. Nel frattempo
  // inizializziamo display e sensori; attendiamo solo l'eventuale tempo residuo.
  const unsigned long gpsStartupElapsed = millis() - gpsSerialStartedAt;
  if (gpsStartupElapsed < 300UL) delay(300UL - gpsStartupElapsed);
  gpsSetRate10Hz();
  gpsConfigureNmeaMinimal();
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "gps", millis());
  loadDisplayThemePreference();
  resetQfeReference();
  qfeStartupResetDone = false;
  lcd.setBrightness(255);
  sensorDataMutex = xSemaphoreCreateMutex();
  i2cMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 4096, NULL, 2, NULL, 0);
  bleStartTask();
  loadScreenPreference();
  loadWifiLogin();   // nome rete e password del WIFI LOG (CHANGE LOGIN), prima di un eventuale avvio del WiFi
  loadLicense();     // codice di attivazione: senza codice valido lo strumento funziona solo in DEMO
  preferences.begin("altimeter", true);
  dualModeEnabled = preferences.getBool("av_dual", false);
  dualSelectionCount = preferences.getInt("dual_count", 0);
  if (dualSelectionCount < 0 || dualSelectionCount > 2) dualSelectionCount = 0;
  dualSelection[0] = (ScreenState)preferences.getInt("dual_page_0", (int)MAIN_ALT_SCREEN);
  dualSelection[1] = (ScreenState)preferences.getInt("dual_page_1", (int)VARIOMETER_SCREEN);
  // Migrazione: l'opzione TIMER del dual e' stata sostituita da GO TO HOME.
  bool dualSelectionMigrated = false;
  for (int i = 0; i < 2; ++i) {
    if (dualSelection[i] == FLIGHT_TIMER_SCREEN) { dualSelection[i] = GO_TO_HOME_SCREEN; dualSelectionMigrated = true; }
  }
  homePositionValid = preferences.getBool("home_valid", false);
  homeLatitude = preferences.getDouble("home_lat", 0.0);
  homeLongitude = preferences.getDouble("home_lon", 0.0);
  hcFormat = constrain(preferences.getInt("hc_fmt", HC_FMT_GARMIN), 0, HC_FMT_COUNT - 1);   // formato coordinate HOME FIELD
  gsYellowKmh = constrain(preferences.getInt("gs_yel", 120), GS_YELLOW_MIN_KMH, GS_YELLOW_MAX_KMH);
  gsRedKmh = constrain(preferences.getInt("gs_red", 160), gsYellowKmh + 10, GS_RED_MAX_KMH);
  gpsSpeedShowKnots = preferences.getBool("gs_kt", false);
  qfeShowFeet = preferences.getBool("qfe_ft", false);
  qfeCircuitLowM = constrain(preferences.getInt("qfe_cl", 250), 0, 8980);
  qfeCircuitHighM = constrain(preferences.getInt("qfe_ch", 350), qfeCircuitLowM + 20, 9000);
  preferences.end();
  if (dualModeEnabled || dualSelectionMigrated) {
    dualModeEnabled = false;
    saveDualPreferences();
  }
  // Avvio: logo per un secondo e mezzo (NON bloccante: GPS, sensori e BLE girano) con il suo
  // tocco (centro -> pagina FIRMWARE); poi, in finishStartupLogo(), eventuale avviso
  // ghiaccio/nebbia e quindi la pagina salvata nelle preferenze.
  // Il logo viene disegnato direttamente (senza redrawCurrentScreen) per non
  // sovrascrivere la pagina salvata.
  startupTargetScreen = currentScreen;
  startupSequenceActive = true;
  currentScreen = AVEA_LOGO_SCREEN;
  drawAveaLogoScreen();
  startupLogoEndMillis = millis() + STARTUP_LOGO_DURATION_MS;
  AVEA_LOGF("[BOOT] %-12s %lu ms\n", "logo", millis());
  playTone(880, 150);
  startupRiskCheckPending = true;
  startupRiskCheckDeadlineMillis = millis() + startupRiskWaitMs();
  // Ripete lo stato PSRAM a fine avvio: le righe iniziali si perdono se il monitor si aggancia tardi.
  AVEA_LOGF("PSRAM: %s, %u byte | heap interno libero %u byte\n", psramFound() ? "TROVATA" : "NON trovata", (unsigned)ESP.getPsramSize(), (unsigned)ESP.getFreeHeap());
  esp_task_wdt_add(NULL);
}

// ========== LOOP ==========
// --- rotazione demo ---
// ===================== DEMO: rotazione di tutte le pagine e avvisi (sketch_test) =====================
// Con la demo attiva lo strumento mostra da solo, 5 s ciascuno, tutte le pagine del giro (anche quelle
// spente in PAGES, tranne WIFI LOG), gli avvisi e le pagine di impostazione. Gli avvisi che aspettano un
// tocco si chiudono da soli come se fossero stati toccati. Niente viene salvato. Un tocco qualsiasi esce
// dalla rotazione e lascia la demo normale sull'ultima pagina del giro mostrata.
enum DemoTourKind : uint8_t {
  TK_PAGE, TK_EXTRA, TK_LOCK, TK_RISK, TK_SUNSET, TK_ICT, TK_RESERVE, TK_ASP_NEAR, TK_ASP_IN, TK_FLIGHT_END,
  TK_QNH, TK_LITERS, TK_FUEL_SETUP, TK_PAGES, TK_METEO_OFS, TK_ALT_OFS, TK_DENSITY
};
struct DemoTourStep { DemoTourKind kind; int16_t arg; };
static const DemoTourStep DEMO_TOUR[] = {
  {TK_PAGE, COMPASS_SCREEN}, {TK_LOCK, 0},
  {TK_PAGE, FIELDS_RADAR_SCREEN}, {TK_PAGE, GO_TO_HOME_SCREEN}, {TK_EXTRA, LUBBER_LINE_SCREEN}, {TK_EXTRA, HOME_FIELD_SCREEN},
  {TK_PAGE, NEAREST_FIELDS_SCREEN}, {TK_ASP_NEAR, 0}, {TK_ASP_IN, 0},
  {TK_PAGE, METEO_SCREEN}, {TK_EXTRA, AIR_DATA_SCREEN}, {TK_RISK, 6}, {TK_RISK, 3}, {TK_DENSITY, 0},
  {TK_PAGE, VARIOMETER_SCREEN}, {TK_PAGE, MAIN_ALT_SCREEN}, {TK_QNH, 0}, {TK_PAGE, QFE_SCREEN},
  {TK_PAGE, GPS_SPEED_SCREEN}, {TK_PAGE, SLIP_SCREEN}, {TK_PAGE, GPS_CLOCK_SCREEN}, {TK_SUNSET, 0},
  {TK_PAGE, AUTOMATIC_TIMER_SCREEN}, {TK_EXTRA, FLIGHT_LOG_SCREEN}, {TK_PAGE, ICT_SCREEN}, {TK_ICT, 0},
  {TK_PAGE, AV_SETUP_SCREEN}, {TK_PAGE, FLIGHT_TIMER_SCREEN},
  {TK_PAGE, FUEL_SCREEN}, {TK_RESERVE, 0}, {TK_LITERS, 0}, {TK_FUEL_SETUP, 0}, {TK_FLIGHT_END, 0},
  {TK_PAGE, FIRMWARE_SCREEN}, {TK_PAGES, 0}, {TK_PAGES, 1}, {TK_PAGES, 2}, {TK_METEO_OFS, 0}, {TK_ALT_OFS, 0},
  {TK_PAGE, GPS_INFO_SCREEN}
};
static const int DEMO_TOUR_COUNT = sizeof(DEMO_TOUR) / sizeof(DEMO_TOUR[0]);
static const unsigned long DEMO_TOUR_STEP_MS = 5000UL;
static int demoTourIdx = -1;
static unsigned long demoTourStepMs = 0;
static ScreenState demoTourLastPage = COMPASS_SCREEN;
static unsigned long demoTourStartMs = 0;

// Spazio aereo finto per l'avviso (solo il tempo del disegno: poi si rimettono i dati veri)
static AspArea demoTourAsp = {0, 0, 0, 0, 0, 0, 0, 2500, ASP_REF_AGL, ASP_REF_AMSL, "D", "CTR", "DEMO CTR"};
static void demoTourDrawAirspace(int level, int distM) {
  AspArea *savedAreas = aspAreas; const int savedCount = aspCount;
  aspAreas = &demoTourAsp; aspCount = 1;
  aspAlertIdx = 0; aspAlertLevel = level; aspAlertDistM = distM; aspAlertVert = 0;
  aspAlertShownMs = millis();
  currentScreen = AIRSPACE_ALERT_SCREEN;
  drawAirspaceAlertScreen();
  aspAreas = savedAreas; aspCount = savedCount;
  aspAlertIdx = -1;   // un ridisegno non mostra uno spazio vero al posto di quello finto
}

// Chiude cio' che il passo aveva aperto (senza salvare nulla)
static void demoTourCleanupStep() {
  if (demoTourIdx < 0) return;
  switch (DEMO_TOUR[demoTourIdx].kind) {
    case TK_LOCK: pageLocked = false; pageLockSplashOn = false; break;
    case TK_RISK: riskWarningForcedPage = 0; riskWarningCycle = false; break;
    case TK_LITERS: fsBootMode = false; break;
    case TK_ASP_NEAR: case TK_ASP_IN: aspAlertIdx = -1; aspAlertLevel = 0; break;
    case TK_PAGES: pageSelectSubPage = 0; break;
    default: break;
  }
}

static void demoTourShowPage(int sc) {   // int: i prototipi automatici di Arduino non vedono ScreenState
  const ScreenState s = (ScreenState)sc;
  const int idx = screenIndexOf(s);
  if (idx >= 0) { currentScreenIndex = idx; demoTourLastPage = s; }
  currentScreen = s;
  redrawCurrentScreen();
}

static void demoTourApply() {
  const DemoTourStep &st = DEMO_TOUR[demoTourIdx];
  lastTouchMillis = millis();
  switch (st.kind) {
    case TK_PAGE: demoTourShowPage(st.arg); break;
    case TK_EXTRA: currentScreen = (ScreenState)st.arg; redrawCurrentScreen(); break;
    case TK_LOCK:
      if (currentScreen != COMPASS_SCREEN) demoTourShowPage(COMPASS_SCREEN);
      pageLocked = true; pageLockSplashOn = true; pageLockSplashUntil = millis() + 1500UL;
      drawPageLockOverlay();
      break;
    case TK_RISK:
      demoRiskTourStep = st.arg;
      demoRiskTourApplyStep();
      riskWarningAutoClose = false; riskWarningCycle = false;
      riskWarningShownMillis = millis();
      riskWarningPreviousScreen = demoTourLastPage;
      currentScreen = RISK_WARNING_SCREEN;
      drawRiskWarningScreen();
      break;
    case TK_SUNSET: {
      // orario del tramonto: quello vero se l'ora e la posizione ci sono, altrimenti uno di prova
      static char demoSunset[8];
      if (demoIntroPrepareSunset()) {
        snprintf(demoSunset, sizeof(demoSunset), "%s", sunsetText);
      } else {
        int h, m, sec, day, mon, yr;
        if (getLocalClock(h, m, sec, day, mon, yr)) {   // ora nota: tramonto fra 30 minuti
          const int t = (h * 60 + m + 30) % 1440;
          snprintf(demoSunset, sizeof(demoSunset), "%02d:%02d", t / 60, t % 60);
        } else {
          snprintf(demoSunset, sizeof(demoSunset), "17:05");
        }
        if (sunsetText[0] < '0' || sunsetText[0] > '9') snprintf(sunsetText, sizeof(sunsetText), "%s", demoSunset);   // per un eventuale ridisegno
      }
      demoSunsetShown = true;
      sunsetPreviousScreen = demoTourLastPage;
      currentScreen = SUNSET_SCREEN;
      drawSunsetScreen(lcd, demoSunset);
      break;
    }
    case TK_ICT:
      ictAlertPreviousScreen = demoTourLastPage;
      ictAlertStartMillis = millis();   // la chiusura automatica (8 s) non scatta prima del passo successivo
      currentScreen = ICT_ALERT_SCREEN;
      drawIctAlertScreen();
      break;
    case TK_RESERVE: currentScreen = FUEL_RESERVE_SCREEN; drawFuelScreen(); break;
    case TK_ASP_NEAR: demoTourDrawAirspace(1, 2400); break;
    case TK_ASP_IN: demoTourDrawAirspace(2, 0); break;
    case TK_FLIGHT_END:
      flightEndPreviousScreen = demoTourLastPage;
      currentScreen = FLIGHT_END_SCREEN;
      drawFlightEndScreen();
      break;
    case TK_QNH:
      qnhBootMode = false; qnhReturnScreen = MAIN_ALT_SCREEN; qnhPendingMeters = altitudeUnitsIsMeters;
      currentScreen = QNH_SETUP_SCREEN;
      drawQnhSetupScreen();
      break;
    case TK_LITERS: openFuelSetupScreen(); fsBootMode = true; drawFuelSetupScreen(); break;
    case TK_FUEL_SETUP: openFuelSetupScreen(); break;
    case TK_PAGES:
      pagesPinMode = false; hardResetPinMode = false; pageSelectSubPage = st.arg;
      currentScreen = PAGE_SELECT_SCREEN;
      drawPageSelectScreen();
      break;
    case TK_METEO_OFS: openMeteoOffsetScreen(); break;
    case TK_ALT_OFS: openAltOffsetScreen(); break;
    case TK_DENSITY: openDensityAltPage(); break;
  }
}

void demoTourStart() {
  demoTourActive = true;
  demoTourIdx = -1;
  demoTourStepMs = 0;
  demoTourStartMs = millis();
  if (isTouching) { ignoreTouchUntilRelease = true; isTouching = false; }   // il rilascio del tocco su DEMO MODE non conta
  demoRiskTourActive = false; demoRiskTourDone = true; demoIntroActive = false;   // niente sequenza iniziale
  demoSunsetShown = true;
}

void demoTourStop() {
  if (!demoTourActive) return;
  demoTourCleanupStep();
  demoTourActive = false;
  demoTourIdx = -1;
  // si resta sull'ultima pagina del giro mostrata (gli avvisi e le impostazioni si chiudono)
  demoTourShowPage(screenIndexOf(currentScreen) >= 0 ? currentScreen : demoTourLastPage);
}

// Nel loop
// Demo senza rotazione (dopo un tocco): gli avvisi che aspettano il tocco si chiudono da soli dopo 5 s
static void demoAlertAutoClose() {
  static int lastScr = -1;
  static unsigned long since = 0;
  const unsigned long now = millis();
  if ((int)currentScreen != lastScr) { lastScr = (int)currentScreen; since = now; return; }
  if (now - since < DEMO_TOUR_STEP_MS || isTouching) return;
  switch (currentScreen) {
    case RISK_WARNING_SCREEN:   dismissRiskWarningScreen(); break;
    case SUNSET_SCREEN:         currentScreen = sunsetPreviousScreen; redrawCurrentScreen(); break;
    case ICT_ALERT_SCREEN:      dismissIctAlertScreen(); break;
    case FUEL_RESERVE_SCREEN:   currentScreen = fuelAlertPrevious; redrawCurrentScreen(); break;
    case AIRSPACE_ALERT_SCREEN: dismissAirspaceAlert(); break;
    case FLIGHT_END_SCREEN:     dismissFlightEndScreen(); break;
    case ONLINE_WX_SCREEN:      break;
    case DENSITY_ALT_SCREEN:    if (daAlertMode) dismissDensityAlt(); break;
    default: return;
  }
  lastScr = -1;
}

void demoTourService() {
  if (demoMode && !demoTourActive) { demoAlertAutoClose(); return; }
  if (!demoTourActive) return;
  if (!demoMode) { demoTourStop(); return; }
  // un tocco NUOVO (iniziato dopo l'avvio della rotazione: il dito di DEMO MODE non conta) la ferma
  if (isTouching && (long)(touchStartAtMillis - demoTourStartMs) > 300L) {
    ignoreTouchUntilRelease = true; isTouching = false;
    demoTourStop();
    playTone(1200, 50);
    return;
  }
  const unsigned long now = millis();
  if (demoTourIdx >= 0 && now - demoTourStepMs < DEMO_TOUR_STEP_MS) return;
  demoTourCleanupStep();
  demoTourIdx = (demoTourIdx + 1) % DEMO_TOUR_COUNT;
  demoTourStepMs = now;
  demoTourApply();
}
// ===================== fine rotazione demo =====================

void loop() {
  // Destinazione comunicata al telefono una volta al secondo: l'app legge queste coordinate
  // e chiede il meteo del campo verso cui stai andando.
  {
    static unsigned long lastDst = 0;
    const unsigned long nowDst = millis();
    if (nowDst - lastDst >= 1000UL) { lastDst = nowDst; bleUpdateDestination(); }
  }
  // Campo scelto a terra: appena arriva il meteo di QUEL campo, la pagina ONLINE WEATHER
  // esce da sola per quattro secondi e poi torna dov'eravamo. In volo non esce mai: la' il
  // pallino del vento pulsa e la pagina la apri tu col tocco, se e quando vuoi.
  if (metWxAutoPending) {
    metWxAutoPending = false;
    float vKmh = 0.0f;
    if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
      vKmh = gps_ground_speed_kmh;
      xSemaphoreGive(sensorDataMutex);
    }
    const bool aTerra = !automaticFlightRunning && !(isfinite(vKmh) && vKmh >= MET_TERRA_KMH);
    if (aTerra && wxAutoUntilMs == 0 &&
        (currentScreen == GO_TO_HOME_SCREEN || currentScreen == FIELDS_RADAR_SCREEN)) {
      wxReturnScreen = (int)currentScreen;
      wxAutoUntilMs = millis() + MET_WX_AUTO_MS;
      if (wxAutoUntilMs == 0) wxAutoUntilMs = 1;
      openOnlineWxPage();
    }
  }
  // Il QNH dal telefono arriva quando la pagina SET QNH e' gia' disegnata, e all'accensione
  // succede sempre: la pagina esce in pochi istanti, il telefono si collega dopo. La pagina si
  // ridisegna solo per un tocco, quindi la riga PHONE non comparirebbe mai e per vederla
  // bisognerebbe uscire e rientrare. Qui si ridisegna quando cambia qualcosa che si vede: la
  // riga che compare o sparisce, il valore proposto, o il colore quando il dato invecchia.
  if (currentScreen == QNH_SETUP_SCREEN) {
    static bool lastShow = false;
    static int lastQ = -32768;
    static bool lastVerde = false;
    const bool show = metQnhValid && metFresh()
                      && fabsf((float)metQnh - (float)seaLevelPressure) >= 0.5f;
    const int q = show ? (int)lroundf((float)metQnh) : -32768;
    const int eta = metAgeMin();
    const bool verde = (eta >= 0 && eta <= 10);
    if (show != lastShow || q != lastQ || (show && verde != lastVerde)) {
      lastShow = show; lastQ = q; lastVerde = verde;
      drawQnhSetupScreen();
    }
  }
  if (wxAutoUntilMs != 0) {
    if (currentScreen != ONLINE_WX_SCREEN) {
      wxAutoUntilMs = 0;        // uscito prima per altra via: non c'e' piu' niente da chiudere
    } else if ((long)(millis() - wxAutoUntilMs) >= 0) {
      wxAutoUntilMs = 0;
      if (wxReturnScreen >= 0) {
        currentScreen = (ScreenState)wxReturnScreen;
        wxReturnScreen = -1;
        for (int i = 0; i < numScreens; ++i) {
          if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
        }
        redrawCurrentScreen();
      }
    } else {
      static unsigned long lastWxDraw = 0;
      const unsigned long nowWx = millis();
      if (nowWx - lastWxDraw >= 500UL) { lastWxDraw = nowWx; drawOnlineWxScreen(); }
    }
  }
  slipBallService();
  slipGyroAidService();   // MPU-6050: zero del giroscopio tenuto giusto dal GPS in volo
  demoTourService();   // demo: rotazione automatica di pagine e avvisi   // TURN & SLIP: lettura MPU-6050 a 100 Hz
  // Reset preferences armato: scade dopo 5 s o uscendo dalla pagina FIRMWARE VERSION
  if (firmwareResetArmedMs != 0 &&
      (currentScreen != FIRMWARE_SCREEN || millis() - firmwareResetArmedMs > FIRMWARE_RESET_CONFIRM_MS)) {
    firmwareResetArmedMs = 0;
    if (currentScreen == FIRMWARE_SCREEN) drawFirmwareScreen();
  }
  esp_task_wdt_reset();
  updateAlertDeferral();
  if (startupSequenceActive) {
    if (currentScreen == AVEA_LOGO_SCREEN) {
      if ((long)(millis() - startupLogoEndMillis) >= 0) finishStartupLogo();
    } else if (currentScreen != FIRMWARE_SCREEN && currentScreen != PAGE_SELECT_SCREEN &&
               currentScreen != ALT_OFFSET_SCREEN && currentScreen != FUEL_SETUP_SCREEN) {
      // pagina cambiata da altro (es. allarme ICT): fine della sequenza di avvio
      startupSequenceActive = false;
      startupRiskCheckPending = true;
      startupRiskCheckDeadlineMillis = millis() + startupRiskWaitMs();
    }
    // Sulla pagina FIRMWARE (aperta toccando il logo) il timer e il controllo
    // rischio restano sospesi: all'uscita (swipe) parte finishStartupLogo().
  }
  // Strumento non attivato: sempre in DEMO (tranne mentre si inserisce il codice di attivazione)
  if (!licenseValid && !demoMode && !startupSequenceActive && currentScreen != LICENSE_SCREEN &&
      !flightTimerRunning && !automaticFlightRunning) {
    startDemoMode(millis());
  }
  checkRuntimeRiskWarning();
  bootWizardLoop();   // accensione: SET QNH -> SET LITERS -> pagina salvata
  checkSunsetAlert();
  checkAirspace();   // avvisi spazi aerei in volo
  if (!qfeStartupResetDone && qfeResetAtMillis != 0 && (long)(millis() - qfeResetAtMillis) >= 0) {
    if (resetQfeReference()) {
      qfeStartupResetDone = true;
      if (currentScreen == QFE_SCREEN) drawQfeScreen();
    } else {
      qfeResetAtMillis = millis() + 500UL;
    }
  }

  if (!demoMode) {
    while (GpsSerial.available() > 0) {
      char c = (char)GpsSerial.read();
      if (c == '\r') continue;
      if (c == '\n') {
        if (nmeaBufferLen > 0) {
          nmeaBuffer[nmeaBufferLen] = '\0';
          if (nmeaHasValidChecksum(nmeaBuffer, nmeaBufferLen)) {
            gpsLastGoodSentenceMs = millis();
            parseNmeaSentence(nmeaBuffer);
          }
        }
        nmeaBufferLen = 0;
        continue;
      }
      if (nmeaBufferLen >= NMEA_BUFFER_MAX) {
        // Buffer pieno senza fine-frase: si perde la frase corrente (troppo
        // lunga/corretta) e si riparte da capo, senza allocazioni dinamiche.
        // Se il carattere che ha causato l'overflow e' un nuovo inizio
        // frase, lo si tiene per non perdere subito anche la successiva.
        nmeaBufferLen = (c == '$') ? 1 : 0;
        if (nmeaBufferLen == 1) nmeaBuffer[0] = c;
        continue;
      }
      nmeaBuffer[nmeaBufferLen++] = c;
    }
  }
  if (!demoMode) gpsWatchdogStep();
  updateDemoData();
  advanceSoftwareClock();
  if (!demoMode && newAltitudeSample) calculateVario();

  uint16_t x = touchX, y = touchY;
  bool touchSampleValid = false;
  bool touched = getTouch(&x, &y, &touchSampleValid);
  if (touchSampleValid) {
    if (touched) {
      if (touchConsecutiveDownSamples < 2) ++touchConsecutiveDownSamples;
      if (touchConsecutiveDownSamples >= 2) touchGestureArmed = true;
    } else if (!isTouching && !ignoreTouchUntilRelease) {
      touchConsecutiveDownSamples = 0;
      touchGestureArmed = false;
    }
  } else {
    // Mantieni lo stato precedente: l'errore I2C non deve simulare un rilascio.
    if (!touchGestureArmed && !touchSkippedNoInt) touchConsecutiveDownSamples = 0;
    touched = isTouching || ignoreTouchUntilRelease;
    x = touchX;
    y = touchY;
  }
  if (touched && touchGestureArmed &&
      currentScreen == FLIGHT_END_SCREEN && demoMode && bleRole != BLE_ROLE_SLAVE &&
      !ignoreTouchUntilRelease &&
      millis() - touchStartAtMillis >= TOUCH_MIN_GESTURE_MS) {
    dismissFlightEndScreen();
    isTouching = true;
    touchX = x;
    touchY = y;
    ignoreTouchUntilRelease = true;
    delay(10);
    return;
  }
  if (touched) {
    if (ignoreTouchUntilRelease) {
      touchX = x;
      touchY = y;
    } else if (!isTouching) {
      isTouching = true;
      touchStartX = x;
      touchStartY = y;
      touchStartAtMillis = millis();
      maxTouchTravel = 0;
      touchCornerX = x;
      touchCornerY = y;
    }
    touchX = x; touchY = y;
    if (!ignoreTouchUntilRelease && y > touchCornerY) { touchCornerX = x; touchCornerY = y; }
    int travelX = abs((int)x - (int)touchStartX);
    int travelY = abs((int)y - (int)touchStartY);
    maxTouchTravel = max(maxTouchTravel, max(travelX, travelY));
    if (!ignoreTouchUntilRelease && touchGestureArmed && isLockableScreen(currentScreen) && maxTouchTravel <= 15 &&
        millis() - touchStartAtMillis >= 2000UL) {   // dito fermo 2 s: blocca / sblocca la pagina
      ignoreTouchUntilRelease = true;   // il rilascio non vale come tocco
      altGsDragTarget = 0; qfeDragEdge = 0; gpsSpeedDragTarget = 0; fuelArcDragArmed = false;   // nessuna soglia spostata
      togglePageLock();
    }
    const bool lockMove = pageLocked && isLockableScreen(currentScreen);   // bloccata: niente soglie trascinate
    if (!ignoreTouchUntilRelease && !lockMove && currentScreen == GPS_SPEED_SCREEN)
      gpsSpeedTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
    if (!ignoreTouchUntilRelease && !lockMove && currentScreen == MAIN_ALT_SCREEN && !dualModeEnabled)
      altGsTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
    if (!ignoreTouchUntilRelease && !lockMove && currentScreen == QFE_SCREEN && !dualModeEnabled)
      qfeTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
    if (!ignoreTouchUntilRelease && currentScreen == HOME_COORDS_SCREEN)
      homeCoordsTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
    if (!ignoreTouchUntilRelease && currentScreen == ALT_OFFSET_SCREEN)
      altOffsetTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
    if (!ignoreTouchUntilRelease && currentScreen == FUEL_SETUP_SCREEN)
      fuelSetupTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
    if (!ignoreTouchUntilRelease && !lockMove && currentScreen == FUEL_SCREEN && !dualModeEnabled)
      fuelArcTouchMove(touchStartX, touchStartY, x, y, touchStartAtMillis | 1UL);
  } else {
    if (ignoreTouchUntilRelease) {
      ignoreTouchUntilRelease = false;
      isTouching = false;
      touchConsecutiveDownSamples = 0;
      touchGestureArmed = false;
    } else if (isTouching) {
      if (!touchGestureArmed || millis() - touchStartAtMillis < TOUCH_MIN_GESTURE_MS) {
        // Un solo campione o un contatto troppo breve viene scartato.
        isTouching = false;
        touchConsecutiveDownSamples = 0;
        touchGestureArmed = false;
      } else if (pageLocked && isLockableScreen(currentScreen)) {
        // Pagina bloccata: swipe e tocchi ignorati (si sblocca tenendo premuto 2 s)
        isTouching = false;
        touchConsecutiveDownSamples = 0;
        touchGestureArmed = false;
      } else {
      int diffX = (int)touchX - (int)touchStartX;
      int diffY = (int)touchY - (int)touchStartY;

      bool isMainScreen = (currentScreen == MAIN_ALT_SCREEN || currentScreen == QFE_SCREEN ||
                           currentScreen == VARIOMETER_SCREEN ||
                           currentScreen == COMPASS_SCREEN || currentScreen == GO_TO_HOME_SCREEN ||
                           currentScreen == GPS_SPEED_SCREEN || currentScreen == GPS_CLOCK_SCREEN ||
                           currentScreen == METEO_SCREEN || currentScreen == AIR_DATA_SCREEN ||
                           currentScreen == AV_SETUP_SCREEN || currentScreen == VARIO_FILTER_SCREEN ||
                           currentScreen == WIFI_LOGS_SCREEN || currentScreen == ICT_SCREEN ||
                           currentScreen == FLIGHT_TIMER_SCREEN || currentScreen == AUTOMATIC_TIMER_SCREEN ||
                           currentScreen == FIRMWARE_SCREEN || currentScreen == AVEA_LOGO_SCREEN ||
                           currentScreen == GPS_INFO_SCREEN || currentScreen == FUEL_SCREEN ||
                           currentScreen == NEAREST_FIELDS_SCREEN || currentScreen == FIELDS_RADAR_SCREEN ||
                           currentScreen == SLIP_SCREEN);
      // SET QNH, WIFI LOG (update firmware) e GPS INFO (BLE): tocco piu' tollerante al movimento del dito
      const bool tapGestureValid = (currentScreen == QNH_SETUP_SCREEN || currentScreen == WIFI_LOGS_SCREEN ||
                                    currentScreen == GPS_INFO_SCREEN || currentScreen == WIFI_LOGIN_SCREEN ||
                                    currentScreen == LICENSE_SCREEN || currentScreen == FIELD_SEARCH_SCREEN)
        ? (maxTouchTravel <= 55 && abs(diffX) < 30 && abs(diffY) < 30)
        : (maxTouchTravel <= 40 && abs(diffX) < 20 && abs(diffY) < 20);
      bool touchHandled = false;

      // Priorita' al pallino del vento prima del riconoscimento degli swipe: bersaglio esteso,
      // con deriva del dito tollerata ma senza catturare gesti lunghi intenzionali.
      if ((currentScreen == FIELDS_RADAR_SCREEN || currentScreen == GO_TO_HOME_SCREEN) &&
          maxTouchTravel <= 55 && abs(diffX) < 35 && abs(diffY) < 35 &&
          windRingTapped(touchStartX, touchStartY)) {
        wxReturnScreen = (int)currentScreen;
        openOnlineWxPage();
        playTone(1200, 50);
        touchHandled = true;
      } else if (currentScreen == FIELD_SEARCH_SCREEN) {
        // SEARCH: tastiera, DEL, risultati (tocco = GO TO); swipe dx/sx torna a NEAREST FIELDS
        fieldSearchTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == LICENSE_SCREEN) {
        // ACTIVATION: tastierino del codice, DEL / OK; swipe dx/sx esce (lo strumento resta in DEMO)
        licenseTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == HOME_COORDS_SCREEN) {
        // COORDS: rulli, N/S-E/W, LAT/LON, SAVE; swipe dx/sx esce senza salvare (niente cambio tema)
        homeCoordsTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == WIFI_LOGIN_SCREEN) {
        // WIFI LOGIN: tastiera, CANCEL / SAVE; swipe dx/sx esce senza salvare
        wifiLoginTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == ALT_OFFSET_SCREEN) {
        // OFFSET: rullo, SAVE; swipe dx/sx = PAGES senza salvare
        altOffsetTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == AVEA_LOGO_SCREEN && compassLogoActive) {
        endCompassLogo();   // logo aperto dalla bussola: tocco o swipe = torna subito alla bussola
        playTone(1500, 80);
        touchHandled = true;
      } else if (currentScreen == QNH_SETUP_SCREEN && qnhBootMode &&
                 abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
        bootWizardSkipQnh();   // SET QNH di accensione: swipe = salta senza salvare
        playTone(1500, 80);
        touchHandled = true;
      } else if (currentScreen == FUEL_SETUP_SCREEN) {
        // FUEL SETUP: rulli, FULL, SAVE, SETUP 1/2 <-> 2/2; swipe dx/sx = FUEL ESTIMATE senza salvare
        fuelSetupTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == FUEL_RESERVE_SCREEN) {
        dismissFuelReserveScreen();   // tocco o swipe: torna alla pagina di prima
        playTone(1500, 80);
        touchHandled = true;
      } else if (currentScreen == FUEL_SCREEN && !dualModeEnabled && fuelArcTouchRelease()) {
        touchHandled = true;   // dito sull'arco: litri regolati (solo a terra)
      } else if (currentScreen == FUEL_SCREEN && tapGestureValid && !dualModeEnabled &&
                 touchStartX >= 90 && touchStartX <= 270 && touchStartY >= 120 && touchStartY <= 270) {
        playTone(1500, 80);
        openFuelSetupScreen();   // tocco al centro di FUEL ESTIMATE: impostazioni
        touchHandled = true;
      } else if (currentScreen == HOME_COORDS_FMT_SCREEN) {
        // COORD FORMAT: tocco su XCSOAR / GARMIN / SKY DEMON / GOOGLE MAPS, swipe dx/sx = HOME FIELD
        homeCoordsFormatTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);
        touchHandled = true;
      } else if (currentScreen == QFE_SCREEN && !dualModeEnabled && qfeTouchRelease()) {
        // QFE stile A: tocco/swipe sull'arco destro = fascia del circuito
        touchHandled = true;
      } else if (currentScreen == MAIN_ALT_SCREEN && !dualModeEnabled &&
          altGsTouchRelease(tapGestureValid, touchStartX, touchStartY)) {
        // ALTM: tocco sul valore GS = km/h <-> nodi, swipe sull'arco destro = soglie colori
        touchHandled = true;
      } else if (currentScreen == GPS_SPEED_SCREEN && gpsSpeedTouchRelease(tapGestureValid, touchStartX, touchStartY)) {
        // GPS SPEED: tocco al centro = km/h <-> nodi, swipe sull'arco = soglie colori
        touchHandled = true;
      } else if (currentScreen == FIRMWARE_SCREEN && !startupSequenceActive && tapGestureValid &&
                 touchStartX >= 228 && touchStartX <= 340 && touchStartY >= 160 && touchStartY <= 232) {
        // Tocco su "TAP TO PAGES" (a destra del QR) nella FIRMWARE VERSION: PIN, poi la pagina nascosta PAGES.
        openPagesPin();
        playTone(1500, 80);
        touchHandled = true;
      } else if (currentScreen == PAGE_SELECT_SCREEN) {
        if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
          // Uscita verso la FIRMWARE VERSION; se la pagina di avvio e' stata disattivata si riparte
          // dalla prima pagina attiva.
          for (int i = 0; i < numScreens; ++i) {
            if (screenOrder[i] == startupTargetScreen) {
              if (!pageIndexEnabled(i)) startupTargetScreen = screenOrder[nextEnabledScreenIndex(i, 1)];
              break;
            }
          }
          if (hardResetPinMode) {   // dalla tastiera di HARD RESET lo swipe torna a PAGES 3/3
            hardResetPinMode = false;
            pinPadTitle = "PIN";
            drawPageSelectScreen();
            playTone(1500, 80);
          } else {
            pagesPinMode = false;
            currentScreen = FIRMWARE_SCREEN;
            drawFirmwareScreen();
            playTone(1500, 80);
          }
        } else if (hardResetPinMode) {
          if (tapGestureValid) hardResetPinTap(touchStartX, touchStartY);
        } else if (pagesPinMode) {
          if (tapGestureValid) pagesPinTap(touchStartX, touchStartY);   // tastiera del PIN
        } else if (tapGestureValid) {
          const int offY = PAGE_SEL_ROW_Y0 + (PAGE_SEL_ROWS - 2) * PAGE_SEL_ROW_H;   // ALT OFFSET, riga di ALTIMETER
          const int pageIdx = pageSelectSubPage == 0 ? pageSelectHitTest(touchStartX, touchStartY)
                            : (pageSelectSubPage == 1 ? pageSelect2HitTest(touchStartX, touchStartY) : -1);
          if (touchStartX >= 90 && touchStartX <= 270 && touchStartY >= 8 && touchStartY <= 58) {
            // clic sulla scritta PAGES 1/3, 2/3, 3/3: passa alla successiva
            pageSelectSubPage = (pageSelectSubPage + 1) % 3;
            drawPageSelectScreen();
            playTone(1500, 80);
          } else if (pageSelectSubPage == 2) {
            if (touchStartX >= HR_BTN_X - 10 && touchStartX <= HR_BTN_X + HR_BTN_W + 10 &&
                touchStartY >= HR_BTN_Y - 10 && touchStartY <= HR_BTN_Y + HR_BTN_H + 10) {
              pinPadReset();
              pinPadTitle = "HARD RESET";
              hardResetPinMode = true;
              drawPinPad();
              playTone(1500, 80);
            } else if (touchStartX >= SLIP_BOX_X - 12 && touchStartX <= SLIP_BOX_X + 200 &&
                       touchStartY >= SLIP_BOX_Y - 4 && touchStartY <= SLIP_BOX_Y + 26) {
              const int si = screenIndexOf(SLIP_SCREEN);   // TURN & SLIP on / off
              const uint32_t newMask = si >= 0 ? (pagesEnabledMask ^ (uint32_t)(1UL << si)) : 0;
              if (newMask != 0) { pagesEnabledMask = newMask; savePagesMask(); drawPageSelectScreen(); playTone(1200, 50); }
            } else if (touchStartX >= NOALERT_BOX_X - 12 && touchStartX <= NOALERT_BOX_X + 220 &&
                       touchStartY >= NOALERT_BOX_Y - 2 && touchStartY <= NOALERT_BOX_Y + 30) {
              alertsMuted = !alertsMuted;   // DON'T SHOW ALERTS on / off
              saveVerticalSwipeSetting();
              drawPageSelectScreen();
              playTone(1200, 50);
            }
          } else if (pageSelectSubPage == 0 && touchStartX >= PAGE_SEL_COL_X[1] - 12 && touchStartX <= 330 &&
                     touchStartY >= offY + PAGE_SEL_ROW_H - 2 && touchStartY <= offY + 2 * PAGE_SEL_ROW_H + 6) {
            lubberEditEnabled = !lubberEditEnabled;   // LUBBER LINE: clic su GO TO HOME attivo / corridoio fisso 1 / +1
            saveVerticalSwipeSetting();
            lastGoToHomeXteState = -1;
            drawPageSelectScreen();
            playTone(1200, 50);
          } else if (pageSelectSubPage == 0 && touchStartX >= PAGE_SEL_COL_X[1] - 12 && touchStartX <= PAGE_SEL_COL_X[1] + 132 &&
                     touchStartY >= PAGE_SEL_ROW_Y0 + 3 * PAGE_SEL_ROW_H && touchStartY < PAGE_SEL_ROW_Y0 + 4 * PAGE_SEL_ROW_H) {
            playTone(1500, 80);
            openMeteoOffsetScreen();   // OFFSET: taratura della temperatura (METEO OFFSET), al posto di ICT
          } else if (pageSelectSubPage == 0 && touchStartX >= PAGE_SEL_COL_X[1] - 12 && touchStartX <= PAGE_SEL_COL_X[1] + 132 &&
              touchStartY >= offY - 4 && touchStartY <= offY + 38) {
            playTone(1500, 80);
            openAltOffsetScreen();   // voce OFFSET: taratura dell'offset dell'altimetro
          } else if (pageSelectSubPage == 1 && pageIdx >= 0 && pageIdx < numScreens &&
                     screenOrder[pageIdx] == FUEL_SCREEN && touchStartX >= PAGE_SEL2_SET_X - 10) {
            playTone(1500, 80);
            openFuelSetupFromPages();   // SET: FUEL SETUP anche con la pagina FUEL ESTIMATE spenta
          } else if (pageSelectSubPage == 1 && pageIdx >= 0 && pageIdx < numScreens &&
                     screenOrder[pageIdx] == G_METER_SCREEN) {
            displayRotated180 = !displayRotated180;   // ROTATE DISPLAY 180: display e touch capovolti
            saveVerticalSwipeSetting();
            applyDisplayRotation();
            drawPageSelectScreen();
            playTone(1200, 50);
          } else if (pageSelectSubPage == 1 && pageIdx == numScreens + 2 && touchStartX >= PAGE_SEL2_CTR_X - 12) {
            aspOnlyCtr = !aspOnlyCtr;   // ONLY CTR on / off: avvisi spazi aerei solo per le CTR
            saveVerticalSwipeSetting();
            drawPageSelectScreen();
            playTone(1200, 50);
          } else if (pageSelectSubPage == 1 && pageIdx == numScreens + 2) {
            const int ictIdx = screenIndexOf(ICT_SCREEN);   // ICT on / off (almeno una pagina deve restare attiva)
            const uint32_t newMask = ictIdx >= 0 ? (pagesEnabledMask ^ (uint32_t)(1UL << ictIdx)) : 0;
            if (newMask != 0) { pagesEnabledMask = newMask; savePagesMask(); drawPageSelectScreen(); playTone(1200, 50); }
          } else if (pageSelectSubPage == 1 && pageIdx == numScreens + 1 && touchStartX >= PAGE_SEL2_GS150_X - 12) {
            gsScale150 = !gsScale150;   // GS 150 on / off: fondo scala GS da 0-150 km/h
            saveVerticalSwipeSetting();
            gsRangeIdx = 0;
            gsUpdateAutoRange(0.0f, false);   // la scala si rifa' con le soglie attuali
            gsScaleNeedsErase = true;
            gpsSpeedLastKey = 0xFFFFFFFFu;
            drawPageSelectScreen();
            playTone(1200, 50);
          } else if (pageSelectSubPage == 1 && pageIdx == numScreens + 1) {
            arcBlockEnabled = !arcBlockEnabled;   // ARCH BLOCK on / off
            saveVerticalSwipeSetting();
            drawPageSelectScreen();
            playTone(1200, 50);
          } else if (pageSelectSubPage == 1 && pageIdx == numScreens) {
            verticalSwipeDisabled = !verticalSwipeDisabled;   // DISABLE VERTICAL SWIPE
            saveVerticalSwipeSetting();
            drawPageSelectScreen();
            playTone(1200, 50);
          } else if (pageIdx >= 0) {
            const uint32_t newMask = pagesEnabledMask ^ (uint32_t)(1UL << pageIdx);
            if (screenOrder[pageIdx] == AV_SETUP_SCREEN) {
              // SETUP nascosta: la riga e' vuota, nessuna azione
            } else if (screenOrder[pageIdx] == FIRMWARE_SCREEN) {
              playTone(600, 80);   // FIRMWARE VERSION: sempre attiva, non si spegne
            } else if (newMask != 0) {   // almeno una pagina deve restare attiva
              pagesEnabledMask = newMask;
              savePagesMask();
              drawPageSelectScreen();
              playTone(1200, 50);
            }
          }
        }
        touchHandled = true;
      } else if (currentScreen == ICT_ALERT_SCREEN) {
        if (bleRole == BLE_ROLE_SLAVE && bleState == BLE_ST_LINKED) bleQueueIctDismiss();
        else if (bleRole != BLE_ROLE_SLAVE) dismissIctAlertScreen();
        touchHandled = true;
      } else if (currentScreen == RISK_WARNING_SCREEN) {
        if (demoRiskTourActive) {
          endDemoIntro();   // tocco o swipe: salta la sequenza iniziale e parte la demo normale
          dismissRiskWarningScreen();
        } else if (!riskPreviewActive && riskWarningStepNext != 0) {
          alertAckSend(alertKindOnScreen());   // letto anche sull'altro strumento
          riskWarningForcedPage = riskWarningStepNext;   // accensione: avviso successivo
          riskWarningStepNext = 0;
          drawRiskWarningScreen();
          playTone(1200, 50);
        } else if (!riskPreviewActive) {
          alertAckSend(alertKindOnScreen());   // letto anche sull'altro strumento
          dismissRiskWarningScreen();
        } else if (tapGestureValid || (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX)) {
          riskPreviewNext();   // giro dalla SENSOR: CARB ICE -> VISIBILITY -> SENSOR
        }
        touchHandled = true;
      } else if (currentScreen == DENSITY_ALT_SCREEN) {
        if (riskPreviewActive) {
          // giro dalla SENSOR: tocco o swipe passano a CARB ICE RISK
          if (tapGestureValid || (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX)) riskPreviewNext();
        } else {
          alertAckSend(1);       // letto anche sull'altro strumento
          dismissDensityAlt();   // avviso di avvio: tocco o swipe continuano l'avvio
          playTone(1200, 50);
        }
        touchHandled = true;
      } else if (currentScreen == ONLINE_WX_SCREEN) {
        if (wxReturnScreen >= 0) {
          // Arrivati col clic sul pallino del vento, o con la pagina uscita da sola: si torna
          // alla pagina di partenza. Il tocco vale anche come chiusura anticipata.
          wxAutoUntilMs = 0;
          currentScreen = (ScreenState)wxReturnScreen;
          wxReturnScreen = -1;
          for (int i = 0; i < numScreens; ++i) {
            if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
          }
          redrawCurrentScreen();
          playTone(1500, 60);
        } else if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
          endRiskPreview();
        } else if (tapGestureValid) {
          riskPreviewNext();   // avanti: DENSITY ALTITUDE
        }
        touchHandled = true;
      } else if (currentScreen == AIR_DATA_SCREEN && tapGestureValid) {
        riskPreviewNext();   // giro dalla SENSOR: AIR DATA -> ONLINE WEATHER
        touchHandled = true;
      } else if (currentScreen == AIRSPACE_ALERT_SCREEN) {
        dismissAirspaceAlert();   // tocco: chiude l'avviso spazio aereo
        playTone(1200, 50);
        touchHandled = true;
      } else if (currentScreen == SUNSET_SCREEN) {
        if (demoRiskTourActive) endDemoIntro();   // tocco durante il sunset della sequenza iniziale
        lastTouchMillis = millis();
        currentScreen = sunsetPreviousScreen;
        redrawCurrentScreen();
        playTone(1200, 50);
        touchHandled = true;
      } else if (currentScreen == FLIGHT_END_SCREEN) {
        if (bleRole != BLE_ROLE_SLAVE) dismissFlightEndScreen();
        else if (bleState == BLE_ST_LINKED) bleQueueCloseFlightEnd();   // il master chiude e lo slave lo segue
        else dismissFlightEndScreen();                                  // senza collegamento: chiusura locale
        touchHandled = true;
      } else if (currentScreen == WIFI_LOGS_SCREEN && fwMode != 0) {
        fwTouchRelease(tapGestureValid, diffX, diffY, touchStartX, touchStartY);   // CHOOSE FW aperto
        touchHandled = true;
      } else if (currentScreen == WIFI_LOGS_SCREEN && tapGestureValid && fwTapOnButton(touchStartX, touchStartY)) {
        playTone(1500, 80);
        fwOpenChooser();
        fwDrawChooser();
        touchHandled = true;
      } else if (currentScreen == WIFI_LOGS_SCREEN && tapGestureValid &&
                 wifiTapOnChangeLogin(touchStartX, touchStartY)) {
        playTone(1500, 80);
        openWifiLoginScreen();   // CHANGE LOGIN: tastiera per nome rete e password
        touchHandled = true;
      } else if (currentScreen == WIFI_LOGS_SCREEN && tapGestureValid &&
                 otaTapOnStatusLine(touchStartX, touchStartY)) {
        playTone(1500, 50);
        otaStartPendingUpdate();
        touchHandled = true;
      } else if (dualModeEnabled && dualSelectionCount == 2 &&
                 abs(diffX) > SWIPE_MIN_X_PX && abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX) {
        dualModeEnabled = false;
        saveDualPreferences();
        for (int i = 0; i < numScreens; ++i) {
          if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
        }
        redrawCurrentScreen();
        playTone(1500, 80);
      } else if (currentScreen == DUAL_MODE_SCREEN &&
                 abs(diffX) > SWIPE_MIN_X_PX && abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX) {
        currentScreen = AV_SETUP_SCREEN;
        drawAvSetupScreen();
        playTone(1500, 80);
      } else if (currentScreen == FIELDS_RADAR_SCREEN &&
                 abs(diffY) > SWIPE_MIN_Y_PX && abs(diffY) > abs(diffX) &&
                 millis() - touchStartAtMillis >= TOUCH_MIN_GESTURE_MS) {
        fieldsRadarZoom(diffY < 0 ? -1 : 1);   // swipe in su: zoom piu' vicino; in giu': piu' lontano
      } else if (currentScreen == NEAREST_FIELDS_SCREEN &&
                 abs(diffY) > SWIPE_MIN_Y_PX && abs(diffY) > abs(diffX) &&
                 millis() - touchStartAtMillis >= TOUCH_MIN_GESTURE_MS) {
        nearestFieldsScroll(diffY < 0 ? 1 : -1);   // swipe in su: campi successivi; in giu': precedenti
      } else if ((currentScreen != COMPASS_SCREEN || verticalSwipeDisabled) &&
                 currentScreen != QNH_SETUP_SCREEN && currentScreen != AIR_DATA_SCREEN &&
                 abs(diffY) > SWIPE_MIN_Y_PX && abs(diffY) > abs(diffX) &&
                 millis() - touchStartAtMillis >= TOUCH_MIN_GESTURE_MS) {
        // Swipe su/giu' ignorato: il tema si cambia solo sulla bussola (e mai con DISABLE VERTICAL SWIPE attivo)
      } else if (currentScreen == COMPASS_SCREEN &&
                 abs(diffY) > SWIPE_MIN_Y_PX && abs(diffY) > abs(diffX) &&
                 millis() - touchStartAtMillis >= TOUCH_MIN_GESTURE_MS) {
        // Bussola: lo swipe verticale avanza il tema: bianco/nero -> colore -> giallo/nero.
        // Il tema cambia solo per la bussola: le altre pagine restano a colori.
        if (compassTheme == THEME_WHITE_BLACK) {
          compassTheme = THEME_COLOR;
        } else if (compassTheme == THEME_COLOR) {
          compassTheme = THEME_YELLOW_BLACK;
        } else {
          compassTheme = THEME_WHITE_BLACK;
        }
        saveDisplayThemePreference();
        redrawCurrentScreen();
        playTone(1200, 50);
      } else if (currentScreen == FLIGHT_LOGS_VIEW_SCREEN) {
        if (abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
          int count = getNumSavedLogs();
          if (count > 0) {
            if (diffX > 0) currentLogViewIndex = (currentLogViewIndex - 1 + count) % count;
            else currentLogViewIndex = (currentLogViewIndex + 1) % count;
            drawFlightLogsViewScreen(currentLogViewIndex);
            playTone(1500, 80);
          }
        } else if (abs(diffX) < 20 && abs(diffY) < 20 && millis() - lastTouchMillis > 300) {
          lastTouchMillis = millis();
          playTone(1200, 50);
          currentScreen = FLIGHT_LOG_SCREEN;
          drawFlightLogScreen();
        }
      } else if (currentScreen == FLIGHT_LOG_SCREEN &&
                 abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
        currentScreen = AUTOMATIC_TIMER_SCREEN;
        for (int i = 0; i < numScreens; ++i) { if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; } }
        saveScreenPreference();
        drawAutomaticFlightTimerScreen();
      } else if (currentScreen == HOME_FIELD_SCREEN &&
                 abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
        homeConfirmPending = HOME_CONFIRM_NONE;
        currentScreen = COMPASS_SCREEN;
        drawCompassScreen(true);
      } else if (currentScreen == AIR_DATA_SCREEN &&
                 abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
        riskPreviewNext();   // swipe su AIR DATA: pagina successiva (DENSITY ALTITUDE)
      } else if (currentScreen == AVEA_LOGO_SCREEN && startupSequenceActive &&
                 abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
        finishStartupLogo();   // swipe sul logo di avvio: salta l'attesa
        playTone(1500, 80);
      } else if (((currentScreen == FIRMWARE_SCREEN && startupSequenceActive) || currentScreen == VARIO_FILTER_SCREEN ||
                  currentScreen == LUBBER_LINE_SCREEN) &&
                 abs(diffY) < SWIPE_CROSS_AXIS_MAX_PX && abs(diffX) > SWIPE_MIN_X_PX) {
        // FILTER (aperta dal vario) e LUBBER LINE non sono nel giro; FIRMWARE ora e' nel giro (swipe normale).
        if (currentScreen == FIRMWARE_SCREEN && startupSequenceActive) {
          // uscita da FIRMWARE durante l'avvio: eventuale avviso ghiaccio/nebbia,
          // poi la pagina salvata nelle preferenze.
          finishStartupLogo();
        } else {
          currentScreen = (currentScreen == FIRMWARE_SCREEN) ? startupTargetScreen
                        : (currentScreen == LUBBER_LINE_SCREEN ? GO_TO_HOME_SCREEN
                        : VARIOMETER_SCREEN);
          for (int i = 0; i < numScreens; ++i) {
            if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
          }
          saveScreenPreference();
          redrawCurrentScreen();
        }
        playTone(1500, 80);
      } else if (isMainScreen && abs(diffY) < max(SWIPE_CROSS_AXIS_MAX_PX, abs(diffX) * 3 / 4)) {   // swipe un po' obliquo accettato
        for (int i = 0; i < numScreens; ++i) {
          if (screenOrder[i] == currentScreen) { currentScreenIndex = i; break; }
        }
        if (diffX > SWIPE_MIN_X_PX) {
          wifiReturnScreen = currentScreen;
          currentScreenIndex = nextEnabledScreenIndex(currentScreenIndex, 1);
          currentScreen = screenOrder[currentScreenIndex];
          if (currentScreen == WIFI_LOGS_SCREEN) startLogWifi();
          saveScreenPreference();
          redrawCurrentScreen();
          playTone(1500, 80);
        } else if (diffX < -SWIPE_MIN_X_PX) {
          wifiReturnScreen = currentScreen;
          currentScreenIndex = nextEnabledScreenIndex(currentScreenIndex, -1);
          currentScreen = screenOrder[currentScreenIndex];
          if (currentScreen == WIFI_LOGS_SCREEN) startLogWifi();
          saveScreenPreference();
          redrawCurrentScreen();
          playTone(1500, 80);
        }
      }

      // Riga "BLE ..." (cambio ruolo OFF/MASTER/SLAVE): area di tocco allargata e tocco piu' tollerante
      if (!touchHandled && currentScreen == GPS_INFO_SCREEN && tapGestureValid &&
          touchStartX >= 40 && touchStartX <= 320 &&
          touchStartY >= 280 && touchStartY <= 359) {
        bleCycleRole();
        touchHandled = true;
      }
      if (!touchHandled) {
        if (currentScreen == FIRMWARE_SCREEN && firmwareResetArmedMs == 0 && maxTouchTravel <= 40 &&
            abs(diffX) < 20 && abs(diffY) < 20 &&
            touchStartX >= firmwareUpdateButtonX &&
            touchStartX < firmwareUpdateButtonX + firmwareUpdateButtonW &&
            touchStartY >= firmwareUpdateButtonY &&
            touchStartY < firmwareUpdateButtonY + firmwareUpdateButtonH) {
            // DEMO MODE: un solo clic avvia la demo (con la demo attiva la ferma)
          const unsigned long tapMillis = millis();
          if (!licenseValid) {
            openLicenseScreen((int)FIRMWARE_SCREEN);   // non attivato: ACTIVATE apre l'inserimento del codice
          } else if (!demoMode) {
            if (startDemoMode(tapMillis) && bleRole == BLE_ROLE_SLAVE) bleQueueDemo(true);    // play anche sul master
          } else {
            stopDemoMode(tapMillis);
            if (bleRole == BLE_ROLE_SLAVE) bleQueueDemo(false);                              // stop anche sul master
          }
          playTone(1500, 80);
          if (currentScreen == FIRMWARE_SCREEN) drawFirmwareScreen();
          touchHandled = true;
        } else if (currentScreen == FIRMWARE_SCREEN && maxTouchTravel <= 40 &&
            abs(diffX) < 20 && abs(diffY) < 20 &&
            touchStartX >= firmwareResetButtonX - FIRMWARE_RESET_HIT_MARGIN_X &&
            touchStartX < firmwareResetButtonX + firmwareResetButtonW + FIRMWARE_RESET_HIT_MARGIN_X &&
            touchStartY >= firmwareResetButtonY - FIRMWARE_RESET_HIT_MARGIN_Y &&
            touchStartY < firmwareResetButtonY + firmwareResetButtonH + FIRMWARE_RESET_HIT_MARGIN_Y) {
          // Primo clic: avvisa (pulsante rosso CONFIRM RESET?). Secondo clic entro 5 s: reset.
          if (firmwareResetArmedMs != 0 && millis() - firmwareResetArmedMs <= FIRMWARE_RESET_CONFIRM_MS) {
            firmwareResetArmedMs = 0;
            playTone(1500, 80);
            resetPreferencesFromFirmware();
          } else {
            firmwareResetArmedMs = millis();
            if (firmwareResetArmedMs == 0) firmwareResetArmedMs = 1;
            playTone(900, 120);
            drawFirmwareScreen();
          }
          touchHandled = true;
        } else if (currentScreen == VARIOMETER_SCREEN && !dualModeEnabled && tapGestureValid &&
                   millis() - lastTouchMillis > 300 &&
                   touchStartX >= 60 && touchStartX <= 300 &&
                   touchStartY >= 270 && touchStartY <= 335) {
          // tocco sulla quota: apre SET QNH; BACK / SAVE torna al variometro
          lastTouchMillis = millis();
          qnhReturnScreen = VARIOMETER_SCREEN;
          qnhPendingMeters = altitudeUnitsIsMeters;
          currentScreen = QNH_SETUP_SCREEN;
          drawQnhSetupScreen();
          playTone(1200, 50);
          touchHandled = true;
        } else if (currentScreen == COMPASS_SCREEN && abs(diffX) < 20 && abs(diffY) < 20 &&
                   touchStartX >= 110 && touchStartX <= 250 && touchStartY >= 252 && touchStartY <= 307) {
          // riquadro A.V.E.A*: mostra il logo per 5 secondi
          playTone(1200, 50);
          showCompassLogo();
        } else if (currentScreen == HOME_FIELD_SCREEN && abs(diffX) < 20 && abs(diffY) < 20) {
          if (homeConfirmPending != HOME_CONFIRM_NONE &&
              millis() - homeConfirmRequestedAt > HOME_CONFIRM_TIMEOUT_MS) {
            homeConfirmPending = HOME_CONFIRM_NONE;
          }
          const int btnY = 200, btnW = 100, btnH = 38;
          const int setX = 68, resetX = 192;
          const int goToW = 160, goToH = 61, goToX = 100, goToY = 257;
          bool inGoTo = touchStartX >= goToX && touchStartX <= goToX + goToW && touchStartY >= goToY && touchStartY <= goToY + goToH;
          bool inSet = touchStartX >= setX && touchStartX <= setX + btnW && touchStartY >= btnY && touchStartY <= btnY + btnH;
          bool inReset = touchStartX >= resetX && touchStartX <= resetX + btnW && touchStartY >= btnY && touchStartY <= btnY + btnH;
          if (inGoTo) {
            homeConfirmPending = HOME_CONFIRM_NONE;
            if (homePositionValid) {
              goToHomeNeedsRedraw = true;
              currentScreen = GO_TO_HOME_SCREEN;
              drawGoToHomeScreen();
              playTone(1500, 80);
            } else {
              drawHomeFieldScreen();
              playTone(600, 80);
            }
          } else if (inSet) {
            if (homeConfirmPending == HOME_CONFIRM_SET) {
              bool gpsValid = false;
              double lat = 0.0, lon = 0.0;
              if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
                gpsValid = gpsDataValid;
                lat = gps_latitude;
                lon = gps_longitude;
                xSemaphoreGive(sensorDataMutex);
              }
              if (gpsValid) {
                applyHomeLocal(true, lat, lon, false);   // HOME FIELD ridisegnata sotto
                if (bleRole == BLE_ROLE_SLAVE) bleQueueHome();   // master e slave con la stessa home
                playTone(1500, 80);
              } else {
                playTone(600, 80);
              }
              homeConfirmPending = HOME_CONFIRM_NONE;
            } else {
              homeConfirmPending = HOME_CONFIRM_SET;
              homeConfirmRequestedAt = millis();
            }
            drawHomeFieldScreen();
          } else if (inReset) {
            // COORDS (al posto di RESET): pagina di inserimento manuale delle coordinate home
            homeConfirmPending = HOME_CONFIRM_NONE;
            openHomeCoordsScreen();
            playTone(1500, 80);
          } else if (touchStartX >= 100 && touchStartX <= 260 && touchStartY >= 70 && touchStartY <= 280) {
            homeConfirmPending = HOME_CONFIRM_NONE;
            currentScreen = COMPASS_SCREEN;
            drawCompassScreen(true);
          } else if (homeConfirmPending != HOME_CONFIRM_NONE) {
            homeConfirmPending = HOME_CONFIRM_NONE;
            drawHomeFieldScreen();
          }
        } else if (currentScreen == GO_TO_HOME_SCREEN && abs(diffX) < 20 && abs(diffY) < 20 &&
                   touchStartX >= 110 && touchStartX <= 250 && touchStartY >= 125 && touchStartY <= 265) {
          // solo la parte centrale: porta a Home Field
          currentScreen = HOME_FIELD_SCREEN;
          drawHomeFieldScreen();
        } else if (currentScreen == GO_TO_HOME_SCREEN && lubberEditEnabled && abs(diffX) < 20 && abs(diffY) < 20 &&
                   touchStartX >= 110 && touchStartX <= 250 && touchStartY >= 30 && touchStartY <= 115) {
          // parte centrale alta: impostazioni del corridoio (LUBBER LINE)
          currentScreen = LUBBER_LINE_SCREEN;
          drawLubberLineScreen();
          playTone(1200, 50);
        } else if (currentScreen == GO_TO_HOME_SCREEN && navGotoActive && abs(diffX) < 20 && abs(diffY) < 20 &&
                   touchStartX >= 110 && touchStartX <= 250 && touchStartY >= 272 && touchStartY <= 335) {
          // GO TO verso un campo: pulsante HOME = di nuovo verso casa (anche sull'altro strumento)
          navCancelGoto();
          bleQueuePrefs();
          drawGoToHomeScreen();
          playTone(1500, 80);
        } else if (currentScreen == NEAREST_FIELDS_SCREEN && abs(diffX) < 20 && abs(diffY) < 20) {
          nearestFieldsTap(touchStartX, touchStartY);
        } else if (currentScreen == FIELDS_RADAR_SCREEN && abs(diffX) < 20 && abs(diffY) < 20) {
          fieldsRadarTap(touchStartX, touchStartY);   // tocco su un campo: GO TO
        } else if (currentScreen == GO_TO_HOME_SCREEN) {
          // altri tocchi su GO TO HOME: ignorati
        } else if (currentScreen != COMPASS_SCREEN && currentScreen != VARIOMETER_SCREEN &&
            currentScreen != FLIGHT_LOGS_VIEW_SCREEN &&
            tapGestureValid && millis() - lastTouchMillis > 300) {
          lastTouchMillis = millis();
          playTone(1200, 50);

          if (currentScreen == VARIO_FILTER_SCREEN) {
            VarioFilterLevel selectedLevel = varioFilterLevel;
            bool selectionTouched = true;
            if (touchStartY >= 88 && touchStartY <= 142) selectedLevel = FILTER_SMOOTH;
            else if (touchStartY >= 152 && touchStartY <= 206) selectedLevel = FILTER_NORMAL;
            else if (touchStartY >= 216 && touchStartY <= 270) selectedLevel = FILTER_REACTIVE;
            else selectionTouched = false;
            if (selectionTouched && selectedLevel != varioFilterLevel) {
              setVarioFilterLevel(selectedLevel);
              kalman_P_vario = 1.0f;
              preferences.begin("altimeter", false);
              preferences.putInt("vario_filter", (int)varioFilterLevel);
              preferences.end();
            }
            drawVarioFilterScreen();
          } else if (currentScreen == LUBBER_LINE_SCREEN) {
            int newWidthIdx = lubberWidthIdx, newYellowIdx = lubberYellowIdx;
            if (touchStartY >= 88 && touchStartY <= 148) {
              for (int i = 0; i < 4; ++i) {
                const int bx = 48 + i * 68;
                if (touchStartX >= bx - 4 && touchStartX <= bx + 64) newWidthIdx = i;
              }
            } else if (touchStartY >= 184 && touchStartY <= 244) {
              for (int i = 0; i < 3; ++i) {
                const int bx = 50 + i * 90;
                if (touchStartX >= bx - 5 && touchStartX <= bx + 85) newYellowIdx = i;
              }
            }
            if (newWidthIdx != lubberWidthIdx || newYellowIdx != lubberYellowIdx) {
              lubberWidthIdx = newWidthIdx;
              lubberYellowIdx = newYellowIdx;
              preferences.begin("altimeter", false);
              preferences.putInt("lub_w", lubberWidthIdx);
              preferences.putInt("lub_y", lubberYellowIdx);
              preferences.end();
              bleQueuePrefs();   // slave: corridoio anche sul master
              lastGoToHomeXteState = -1;
              drawLubberLineScreen();
            }
          } else if (currentScreen == MAIN_ALT_SCREEN) {
            if (touchStartX >= SETQNH_BOX_X - 25 && touchStartX <= SETQNH_BOX_X + SETQNH_BOX_W + 25 &&
                touchStartY >= SETQNH_BOX_Y - 12 && touchStartY <= 359) {   // SET QNH in fondo al display
              qnhReturnScreen = MAIN_ALT_SCREEN;
              currentScreen = QNH_SETUP_SCREEN; qnhPendingMeters = altitudeUnitsIsMeters; drawQnhSetupScreen();
            }
          } else if (currentScreen == QNH_SETUP_SCREEN) {
            if (touchStartX >= 20 && touchStartX <= 175 && touchStartY >= 65 && touchStartY <= 155) { seaLevelPressure = max(QNH_MIN_HPA, (float)seaLevelPressure - 1.0f); if (bleRole == BLE_ROLE_SLAVE) bleQueueQnh(seaLevelPressure); drawQnhSetupScreen(); }
            else if (touchStartX >= 185 && touchStartX <= 340 && touchStartY >= 65 && touchStartY <= 155) { seaLevelPressure = min(QNH_MAX_HPA, (float)seaLevelPressure + 1.0f); if (bleRole == BLE_ROLE_SLAVE) bleQueueQnh(seaLevelPressure); drawQnhSetupScreen(); }
            else if (touchStartX >= 20 && touchStartX <= 175 && touchStartY >= 200 && touchStartY <= 265) { qnhPendingMeters = true; drawQnhSetupScreen(); }
            else if (touchStartX >= 185 && touchStartX <= 340 && touchStartY >= 200 && touchStartY <= 265) { qnhPendingMeters = false; drawQnhSetupScreen(); }
            // Tocco sulla riga "PHONE ...": accetta il QNH proposto dal telefono.
            else if (touchStartY >= 331 && touchStartX >= 85 && touchStartX <= 275
                     && metQnhValid && metFresh()) {
              seaLevelPressure = constrain((float)metQnh, (float)QNH_MIN_HPA, (float)QNH_MAX_HPA);
              if (bleRole == BLE_ROLE_SLAVE) bleQueueQnh(seaLevelPressure);
              playTone(1500, 60);
              drawQnhSetupScreen();
            }
            else if (touchStartX >= 55 && touchStartX <= 305 && touchStartY >= 270 && touchStartY <= 330) {
              qnhSetupSave();
              // Il cambio/salvataggio del QNH non deve azzerare il QFE:
              // il riferimento cambia solo con il tasto RESET QFE.
              if (qnhBootMode) { bootWizardAfterQnh(); playTone(1500, 80); }   // NEXT all'accensione: SET LITERS
              else {
              currentScreen = qnhReturnScreen;   // ALT o VARIOMETER, da dove si e' aperta SET QNH
              for (int k = 0; k < numScreens; ++k) {
                if (screenOrder[k] == currentScreen) { currentScreenIndex = k; break; }
              }
              redrawCurrentScreen();
              }
            }
          } else if (currentScreen == AV_SETUP_SCREEN) {
            const int dualX = 25, dualY = 90, dualW = 300, dualH = 86;
            const int tw = 65, spacing = 8, startX = 33;
            bool changed = false;
            if (touchStartX >= dualX && touchStartX <= dualX + dualW &&
                touchStartY >= dualY && touchStartY <= dualY + dualH) {
              currentScreen = DUAL_MODE_SCREEN;
              drawDualModeScreen();
            } else if (touchStartY >= 272 && touchStartY <= 335 &&
                       touchStartX >= 80 && touchStartX <= 280) {
              // EXIT: torna alla pagina VARIOMETER
              currentScreen = VARIOMETER_SCREEN;
              for (int k = 0; k < numScreens; ++k) {
                if (screenOrder[k] == currentScreen) { currentScreenIndex = k; break; }
              }
              saveScreenPreference();
              redrawCurrentScreen();
            } else if (touchStartY >= 200 && touchStartY <= 271 &&
                       touchStartX >= 15 && touchStartX <= 345) {
              // Zona di tocco ampia: colonne contigue (nessuno spazio morto tra i riquadri)
              int i = ((int)touchStartX - startX + spacing / 2) / (tw + spacing);
              if (touchStartX < startX) i = 0;
              if (i > 3) i = 3;
              avThresholdIndex = i;
              varioUpThreshold = avLiftThresholds[i];
              varioDownThreshold = -varioUpThreshold;
              changed = true;
            }
            if (changed) {
              preferences.begin("altimeter", false);
              preferences.putInt("av_threshold", avThresholdIndex);
              preferences.end();
              drawAvSetupScreen();
            }
          } else if (currentScreen == DUAL_MODE_SCREEN) {
            for (int i = 0; i < 8; ++i) {
              int optionX = dualOptionX(i), optionY = dualOptionY(i);
              if (touchStartX >= optionX && touchStartX <= optionX + 165 &&
                  touchStartY >= optionY && touchStartY <= optionY + 40) {
                ScreenState page = dualPageOptions[i];
                if (isDualSelected(page)) {
                  if (dualSelectionCount > 0 && dualSelection[0] == page) {
                    dualSelection[0] = dualSelection[1];
                    dualSelection[1] = MAIN_ALT_SCREEN;
                  }
                  if (dualSelectionCount > 0) dualSelectionCount--;
                  dualModeEnabled = false;
                  saveDualPreferences();
                  drawDualModeScreen();
                } else if (dualSelectionCount < 2) {
                  dualSelection[dualSelectionCount++] = page;
                  if (dualSelectionCount == 2) {
                    dualModeEnabled = true;
                    dualActiveIndex = 0;
                    dualLastSwitchMillis = millis();
                    saveDualPreferences();
                    currentScreen = dualSelection[0];
                    redrawCurrentScreen();
                  } else {
                    drawDualModeScreen();
                  }
                }
                break;
              }
            }
          } else if (currentScreen == QFE_SCREEN) {
            // "QFE" = cambia stile A/B/C, valore = metri/piedi, RESET QFE = azzera
            qfeHandleTap(touchStartX, touchStartY);
          } else if (currentScreen == VARIOMETER_SCREEN) {
            currentScreenIndex = 0; currentScreen = MAIN_ALT_SCREEN; saveScreenPreference(); drawMainScreen();
          } else if (currentScreen == SLIP_SCREEN) {
            slipHandleTap(touchStartX, touchStartY);   // due tocchi sull'ampolla = SET 0
          } else if (currentScreen == GPS_CLOCK_SCREEN) {
            // Fuso orario automatico dalla posizione GPS: il tocco non cambia piu' il fuso.
          } else if (currentScreen == METEO_SCREEN) {
            if (touchStartX >= 85 && touchStartX <= 275 && touchStartY >= 275 && touchStartY <= 320) {
              // tocco sulla parte bassa: giro AIR DATA -> CARB ICE RISK -> VISIBILITY (3 s ciascuna), swipe = METEO
              startRiskPreview();
            }   // il tocco sul valore al centro della pagina non fa piu' nulla
          } else if (currentScreen == AUTOMATIC_TIMER_SCREEN) {
            if (touchStartX >= 80 && touchStartX <= 280 && touchStartY >= 275 && touchStartY <= 325) {
              currentScreen = FLIGHT_LOG_SCREEN; drawFlightLogScreen();
            }
          } else if (currentScreen == FLIGHT_LOG_SCREEN) {
            const int slMargin = 30;
            const int slX0 = SAVED_LOGS_BTN_X - slMargin;
            const int slX1 = SAVED_LOGS_BTN_X + SAVED_LOGS_BTN_W + slMargin;
            const int slY0 = SAVED_LOGS_BTN_Y - slMargin;
            const int slY1 = SAVED_LOGS_BTN_Y + SAVED_LOGS_BTN_H + slMargin;
            if (touchStartX >= slX0 && touchStartX <= slX1 &&
                touchStartY >= slY0 && touchStartY <= slY1) {
              int count = getNumSavedLogs();
              if (count > 0) {
                currentLogViewIndex = 0;
                currentScreen = FLIGHT_LOGS_VIEW_SCREEN;
                drawFlightLogsViewScreen(0);
              } else {
                currentScreen = FLIGHT_LOGS_VIEW_SCREEN;
                drawNoLogsViewScreen();
              }
            }
          } else if (currentScreen == ICT_SCREEN) {
            if (demoMode) {
              touchHandled = true; // ICT bloccato a 70 s durante la demo
            } else {
            bool settingChanged = false;
            if (touchStartY >= 205 && touchStartY <= 285 && touchStartX <= 145) {
              ictMinutes = max(1, ictMinutes - 1);
              settingChanged = true;
            } else if (touchStartY >= 205 && touchStartY <= 285 && touchStartX >= 215) {
              ictMinutes = min(30, ictMinutes + 1);
              settingChanged = true;
            } else if (touchStartY >= 282 && touchStartY <= 344 && touchStartX >= 55 && touchStartX <= 305) {
              ictEnabled = !ictEnabled;
              ictStartMillis = millis();
              settingChanged = true;
            }
            if (settingChanged) {
              preferences.begin("altimeter", false);
              preferences.putInt("ict_minutes", ictMinutes);
              preferences.putBool("ict_enabled", ictEnabled);
              preferences.end();
              if (bleRole == BLE_ROLE_SLAVE) bleQueueIct();
              drawIctScreen();
            }
            }
          } else if (currentScreen == FLIGHT_TIMER_SCREEN) {
            if (touchStartY >= 215 && touchStartY <= 300 && touchStartX >= 30 && touchStartX <= 125) {
              if (!flightTimerRunning) {
                unsigned long now = millis();
                flightTimerRunning = true;
                flightTimerStartMillis = now;
                getLocalTimeText(flightStartUtc, sizeof(flightStartUtc));
              }
              drawFlightTimerScreen();
            } else if (touchStartY >= 215 && touchStartY <= 300 && touchStartX >= 132 && touchStartX <= 227) {
              if (flightTimerRunning) {
                flightTimerAccumMillis += millis() - flightTimerStartMillis;
                flightTimerRunning = false;
                flightStatsLastUpdateMillis = 0;
                getLocalTimeText(flightStopUtc, sizeof(flightStopUtc));
              }
              drawFlightTimerScreen();
            } else if (touchStartY >= 215 && touchStartY <= 300 && touchStartX >= 235 && touchStartX <= 330) {
              flightTimerRunning = false;
              flightTimerAccumMillis = 0;
              if (!automaticFlightRunning) clearFlightLogMetrics();
              snprintf(flightStartUtc, sizeof(flightStartUtc), "--:--:--");
              snprintf(flightStopUtc, sizeof(flightStopUtc), "--:--:--");
              drawFlightTimerScreen();
            }
          }
        }
      }
      isTouching = false;
      touchConsecutiveDownSamples = 0;
      touchGestureArmed = false;
      }
    }
  }

  bleApplyRemoteCommands(); // solo MASTER: comandi ricevuti dallo slave (chiudi fine volo, QNH, demo)
  alertAckLoop();           // avvisi letti su master e slave insieme
  pageLockLoop();           // blocco pagina: lucchetto
  bleApplyRemoteQnh();      // solo SLAVE: QNH impostato sul master
  bleApplyRemoteTimezone(); // solo SLAVE: fuso UTC impostato sul master
  bleApplyRemoteArc();      // solo SLAVE: soglie arco velocita' (ALTIMETER / GPS SPEED) del master
  bleApplyRemotePrefs();    // solo SLAVE: unita', QFE e corridoio del master
  updateAutoTimezonePosition();   // fuso orario automatico dalla posizione (ogni 5 s)
  bleApplyRemoteHome();     // solo SLAVE: home impostata sul master
  bleApplyRemoteFuel();     // solo SLAVE: carburante impostato sul master
  bleApplyRemoteMetSlip();  // solo SLAVE: meteo del telefono e sbandometro, che stanno sul master
  bleApplyRemoteTimer(); // timer automatico dello SLAVE, anche in demo
  bleApplyRemoteLog();
  bleApplyRemoteStatus();
  bleApplyRemoteGps();  // solo SLAVE: usa il GPS del master se il proprio non ha fix
  if (!demoMode && (lastRmcMillis == 0 || lastGgaMillis == 0 ||
                    millis() - lastRmcMillis > 3000 || millis() - lastGgaMillis > 3000)) {
    if (xSemaphoreTake(sensorDataMutex, (TickType_t)10) == pdTRUE) {
      gpsRmcStatusValid = false;
      gpsDataValid = false;
      gps_fix_quality = 0;
      gps_ground_speed_kmh = 0.0f;
      xSemaphoreGive(sensorDataMutex);
    }
  }

  handleVarioAudio();
  handleIctAlarm();
  handleLogWifi();
  updateAltOffsetScreen();    // rullo e quota della pagina OFFSET
  statusBlinkStep();          // FOG / SERIOUS RISK / HAZE POSSIBLE / MUCH LONGER TAKE-OFF lampeggianti
  updateFuel();               // stima carburante, avviso RESERVE, ridisegno FUEL
  updateCompassLogo();        // logo A.V.E.A* dalla bussola: 5 s
  applyFlightPageLock();      // volo reale: niente impostazioni e pagine nascoste
  updateLowGsState();         // LOW GS: armato a 100 km/h dopo il decollo, attivo sotto 65 km/h
  updateHomeCoordsScreen();   // inerzia dei rulli della pagina COORDS (home)
  updateDensityAltScreen();   // quota densita': valori ogni 2 s
  updateChipTemperature();    // termometro interno: compensazione del calore dello strumento
  updateOatProbe();           // sonda esterna DS18B20 (se collegata)

  // Lancetta che scorre su GPS SPEED e sull'arco dell'ALTIMETER: aggiornata fino a 40 volte al secondo
  // (il resto delle pagine resta a 10 volte al secondo). Se il valore non cambia non si ridisegna niente.
  static unsigned long lastNeedleAnim = 0;
  if (millis() - lastNeedleAnim >= 25UL) {
    lastNeedleAnim = millis();
    if (currentScreen == GPS_SPEED_SCREEN) updateGpsGroundSpeed();
    else if (currentScreen == MAIN_ALT_SCREEN) updateMainGsGauge(false);
  }

  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 100) {
    lastUpdate = millis();
    updateFlightTimer();
    updateIctTimer();
    updateDualMode();
    switch (currentScreen) {
      case MAIN_ALT_SCREEN: updateMainData(); break;
      case QFE_SCREEN: updateQfeData(); break;
      case VARIOMETER_SCREEN: updateVarioData(); break;
      case COMPASS_SCREEN: drawCompassScreen(false); break;
      case GPS_SPEED_SCREEN: updateGpsGroundSpeed(); break;
      case GPS_CLOCK_SCREEN: updateGpsClock(); break;
      case GO_TO_HOME_SCREEN: updateGoToHomeScreen(); break;
      case NEAREST_FIELDS_SCREEN: updateNearestFieldsScreen(false); break;
      case FIELD_SEARCH_SCREEN: updateFieldSearchScreen(); break;
      case FIELDS_RADAR_SCREEN: updateFieldsRadarScreen(false); break;
      case SLIP_SCREEN: drawSlipScreen(false); break;
      case METEO_SCREEN: updateMeteoData(); break;
      case AIR_DATA_SCREEN: updateAirData(); break;
      case ICT_ALERT_SCREEN: drawIctAlertScreen(); break;
      case GPS_INFO_SCREEN: updateGpsInfoScreen(); break;
      default: break;
    }
  }
  // Pausa minima a fine giro (era 10 ms): la lancetta a 40 Hz arriva puntuale e il giro
  // lascia comunque spazio agli altri task (watchdog, Wi-Fi, BLE).
  delay(1);
}
