module.exports = async function (context, req) {
  context.log("Sensor function called.");

  try {
    const body = req.body;

    if (!body) {
      context.res = {
        status: 400,
        body: {
          ok: false,
          error: "Missing JSON body"
        }
      };
      return;
    }

    const payload = {
      deviceId: body.deviceId || "nano-esp32",
      timestamp: body.timestamp || new Date().toISOString(),

      // session / mode info
      mode: body.mode || "UNKNOWN",
      sessionType: body.sessionType || "live",

      // legacy / live fields
      ppg: body.ppg ?? null,
      imuX: body.imuX ?? null,
      imuY: body.imuY ?? null,
      imuZ: body.imuZ ?? null,
      compressionRate: body.compressionRate ?? null,
      compressionDepth: body.compressionDepth ?? null,
      recoil: body.recoil ?? null,
      status: body.status || "Live",

      // training summary fields
      sampleCount: body.sampleCount ?? null,
      sessionDurationMs: body.sessionDurationMs ?? null,
      avgWeight: body.avgWeight ?? null,
      avgMaxWeight: body.avgMaxWeight ?? null,
      avgCompRate: body.avgCompRate ?? null,
      avgPPG: body.avgPPG ?? null,
      avgBPM: body.avgBPM ?? null,
      avgPitch: body.avgPitch ?? null,
      avgRoll: body.avgRoll ?? null
    };

    context.bindings.actions = {
      actionName: "sendToAll",
      data: JSON.stringify(payload),
      dataType: "json"
    };

    context.res = {
      status: 200,
      headers: {
        "Content-Type": "application/json"
      },
      body: {
        ok: true,
        received: payload
      }
    };
  } catch (error) {
    context.log.error("Sensor function error:", error);

    context.res = {
      status: 500,
      body: {
        ok: false,
        error: "Server error"
      }
    };
  }
};
