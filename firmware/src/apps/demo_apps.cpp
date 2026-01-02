#include "demo_apps.h"

DemoApps::DemoApps(SemaphoreHandle_t mutex) : HassApps(mutex)
{
    HassApps::sync(cJSON_Parse(R"([
        {"app_slug": "climate","app_id": "climate.demo","friendly_name": "Climate","area": "Demo","menu_color": "#ffffff","entity_id": "climate"},
        {"app_slug": "blinds","app_id": "blinds.demo","friendly_name": "Blinds","area": "Demo","menu_color": "#ffffff","entity_id": "blinds"},
        {"app_slug": "light_switch","app_id": "ceiling.demo","friendly_name": "Ceiling","area": "Demo","menu_color": "#ffffff","entity_id": "ceiling"},
        {"app_slug": "light_dimmer","app_id": "workbench.demo","friendly_name": "Workbench","area": "Demo","menu_color": "#ffffff","entity_id": "workbench"}
    ])"));
    menu->setMenuName("Demo");
}

void DemoApps::handleNavigationEvent(NavigationEvent event)
{
    switch (event)
    {
    case NavigationEvent::LONG:
        if (active_id == MENU)
        {
            os_config_notifier->setOSMode(ONBOARDING);
            return;
        }
        break;
    }
    Apps::handleNavigationEvent(event);
}

void DemoApps::setOSConfigNotifier(OSConfigNotifier *os_config_notifier)
{
    this->os_config_notifier = os_config_notifier;
}
