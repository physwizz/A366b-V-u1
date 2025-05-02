_platform_map = {
    "parrot-tuivm": {
        "dtb_list": [
            # keep sorted
            {"name": "parrot-vm-rumi.dtb"},
            {"name": "parrot-vm-atp.dtb"},
            {"name": "parrot-vm-idp.dtb"},
            {"name": "parrot-vm-idp-wcn3990.dtb"},
            {"name": "parrot-vm-idp-wcn3990-amoled-rcm.dtb"},
            {"name": "parrot-vm-idp-wcn6750-amoled.dtb"},
            {"name": "parrot-vm-idp-wcn6750-amoled-rcm.dtb"},
            {"name": "parrot-vm-qrd.dtb"},
            {"name": "parrot-vm-qrd-wcn6750.dtb"},
            {"name": "ravelin-vm-rumi.dtb"},
            {"name": "ravelin-vm-atp.dtb"},
            {"name": "ravelin-vm-idp.dtb"},
            {"name": "ravelin-vm-idp-wcn3988.dtb"},
            {"name": "ravelin-vm-idp-wcn3950-amoled-rcm.dtb"},
            {"name": "ravelin-vm-qrd.dtb"},
        ],
    },
    "parrot": {
        "dtb_list": [
            # keep sorted
            {"name": "parrotp.dtb"},
            {"name": "parrot-sg.dtb"},
            {"name": "parrotp-sg.dtb"},
            {"name": "parrot-4gb.dtb"},
        ],
        "dtbo_list": [],
	"NOTE: excluded dtbo_list" : [
            # keep sorted
            {"name": "parrot-rumi-overlay.dtbo"},
            {"name": "parrot-atp-overlay.dtbo"},
            {"name": "parrot-idp-overlay.dtbo"},
            {"name": "parrot-idp-wcn3990-overlay.dtbo"},
            {"name": "parrot-idp-wcn3990-amoled-rcm-overlay.dtbo"},
            {"name": "parrot-idp-wcn6750-amoled-rcm-overlay.dtbo"},
            {"name": "parrot-idp-wcn6750-amoled-overlay.dtbo"},
            {"name": "parrot-idp-nopmi-overlay.dtbo"},
            {"name": "parrot-idp-pm8350b-overlay.dtbo"},
            {"name": "parrot-qrd-overlay.dtbo"},
            {"name": "parrot-qrd-wcn6750-overlay.dtbo"},
            {"name": "parrot-qrd-nopmi-overlay.dtbo"},
            {"name": "parrot-qrd-pm8350b-overlay.dtbo"},
            {"name": "parrot-idp-4gb-overlay.dtbo"},
            {"name": "parrot-idp-wcn3990-4gb-overlay.dtbo"},
            {"name": "parrot-idp-wcn3990-amoled-rcm-4gb-overlay.dtbo"},
            {"name": "parrot-idp-wcn6750-amoled-rcm-4gb-overlay.dtbo"},
            {"name": "parrot-idp-wcn6750-amoled-4gb-overlay.dtbo"},
            {"name": "parrot-qrd-4gb-overlay.dtbo"},
            {"name": "parrot-qrd-wcn6750-4gb-overlay.dtbo"},
            {"name": "parrot-idp-wcn6755-amoled-rcm-overlay.dtbo"},
            {"name": "parrot-idp-wcn6755-overlay.dtbo"},
            {"name": "parrot-qrd-wcn6755-overlay.dtbo"},
            {"name": "parrot-idp-wcn6755-pm8350b-overlay.dtbo"},
            {"name": "parrot-idp-wcn6755-nopmi-overlay.dtbo"},
            {"name": "ravelin-rumi-overlay.dtbo"},
            {"name": "ravelin-atp-overlay.dtbo"},
            {"name": "ravelin-idp-overlay.dtbo"},
            {"name": "ravelin-idp-wcn3950-amoled-rcm-overlay.dtbo"},
            {"name": "ravelin-qrd-overlay.dtbo"},
            {"name": "ravelin-idp-wcn3988-4gb-overlay.dtbo"},
            {"name": "ravelin-qrd-4gb-overlay.dtbo"},
        ],
    },
}

def _get_dtb_lists(target, dt_overlay_supported):
    if not target in _platform_map:
        fail("{} not in device tree platform map!".format(target))

    ret = {
        "dtb_list": [],
        "dtbo_list": [],
    }

    for dtb_node in [target] + _platform_map[target].get("binary_compatible_with", []):
        ret["dtb_list"].extend(_platform_map[dtb_node].get("dtb_list", []))
        if dt_overlay_supported:
            ret["dtbo_list"].extend(_platform_map[dtb_node].get("dtbo_list", []))
        else:
            # Translate the dtbo list into dtbs we can append to main dtb_list
            for dtb in _platform_map[dtb_node].get("dtb_list", []):
                dtb_base = dtb["name"].replace(".dtb", "")
                for dtbo in _platform_map[dtb_node].get("dtbo_list", []):
                    if not dtbo.get("apq", True) and dtb.get("apq", False):
                        continue

                    dtbo_base = dtbo["name"].replace(".dtbo", "")
                    ret["dtb_list"].append({"name": "{}-{}.dtb".format(dtb_base, dtbo_base)})

    return ret

def get_dtb_list(target, dt_overlay_supported = True):
    return [dtb["name"] for dtb in _get_dtb_lists(target, dt_overlay_supported).get("dtb_list", [])]

def get_dtbo_list(target, dt_overlay_supported = True):
    return [dtb["name"] for dtb in _get_dtb_lists(target, dt_overlay_supported).get("dtbo_list", [])]
