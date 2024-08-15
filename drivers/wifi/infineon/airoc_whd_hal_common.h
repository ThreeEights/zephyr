
#include <whd.h>
#include <zephyr/device.h>

/** Defines the amount of stack memory available for the wifi thread. */
#if !defined(CY_WIFI_THREAD_STACK_SIZE)
#define CY_WIFI_THREAD_STACK_SIZE (5120)
#endif

/** Defines the priority of the thread that services wifi packets. Legal values are defined by the
 *  RTOS being used.
 */
#if !defined(CY_WIFI_THREAD_PRIORITY)
#define CY_WIFI_THREAD_PRIORITY (CY_RTOS_PRIORITY_HIGH)
#endif

/** Map Zephyr country codes (2 octets) to whd_country_code_t.
 * 
*/
#if defined(CONFIG_AIROC_WIFI_COUNTRY)
#if defined(CONFIG_BIG_ENDIAN)
#define AIROC_MAP_COUNTRY_CODE(code) ( ( code[1] ) + ( code[0] << 8 ) )
#else
#define AIROC_MAP_COUNTRY_CODE(code) ( ( code[0] ) + ( code[1] << 8 ) )
#endif
#endif

/** Defines the country this will operate in for wifi initialization parameters. See the
 *  wifi-host-driver's whd_country_code_t for legal options.
 */
#if !defined(CY_WIFI_COUNTRY)
#define CY_WIFI_COUNTRY (WHD_COUNTRY_AUSTRALIA)
#endif

/** Defines the priority of the interrupt that handles out-of-band notifications from the wifi
 *  chip. Legal values are defined by the MCU running this code.
 */
#if !defined(CY_WIFI_OOB_INTR_PRIORITY)
#define CY_WIFI_OOB_INTR_PRIORITY (2)
#endif

/** Defines whether to use the out-of-band pin to allow the WIFI chip to wake up the MCU. */
#if defined(CY_WIFI_HOST_WAKE_SW_FORCE)
#define CY_USE_OOB_INTR (CY_WIFI_HOST_WAKE_SW_FORCE)
#else
#define CY_USE_OOB_INTR (1u)
#endif /* defined(CY_WIFI_HOST_WAKE_SW_FORCE) */

#define CY_WIFI_HOST_WAKE_IRQ_EVENT GPIO_INT_TRIG_LOW
#define DEFAULT_OOB_PIN             (0)
#define WLAN_POWER_UP_DELAY_MS      (250)
#define WLAN_CBUCK_DISCHARGE_MS     (10)

extern whd_resource_source_t resource_ops;

int airoc_wifi_power_on(const struct device *dev);