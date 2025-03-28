package org.firstinspires.ftc.robotcontroller.internal;

import static fi.iki.elonen.NanoHTTPD.newFixedLengthResponse;

import android.os.Environment;

import com.google.blocks.ftcrobotcontroller.util.FileManager;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingMode;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingModeManager;
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil;
import org.firstinspires.ftc.robotserver.internal.webserver.NoCachingWebHandler;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers;
import org.firstinspires.ftc.robotserver.internal.webserver.SessionParametersGenerator;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

import fi.iki.elonen.NanoHTTPD;

public class InputShapingW implements ProgrammingMode {
    private static final String URI_SERVER = "/tuning/input_shaping";
    public ProgrammingModeManager programmingModeManager;
    public static class Server implements WebHandler {

        private static final String PARAM_FM_NAME = "fmname";

        @Override
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws IOException, NanoHTTPD.ResponseException {

            String fmName = getFirstNamedParameter(session, PARAM_FM_NAME);
            if (fmName != null) {
                try {
                    String json = FileManager.valueOf(fmName).fetchFiles();
                    return NoCachingWebHandler.setNoCache(session,
                            newFixedLengthResponse(NanoHTTPD.Response.Status.OK, NanoHTTPD.MIME_PLAINTEXT, json));
                } catch (Exception e) {
                    e.printStackTrace();
                    return newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, NanoHTTPD.MIME_PLAINTEXT,
                            "Internal Error");
                }
            }
            return newFixedLengthResponse(
                    NanoHTTPD.Response.Status.BAD_REQUEST, NanoHTTPD.MIME_PLAINTEXT,"parameter requried");

        }
    }
    private WebHandler decorateWithLogging(WebHandler handler) {

        return programmingModeManager.decorate(false, handler);
    }

    @Override
    public void register(ProgrammingModeManager manager) {
        programmingModeManager = manager;
        manager.register(URI_SERVER,           decorateWithLogging(decorateWithParms( new Server())));
    }

    static String getFirstNamedParameter(NanoHTTPD.IHTTPSession session, String name) {
        final Map<String, List<String>> parameters = session.getParameters();
        if (!parameters.containsKey(name)) return null;
        return parameters.get(name).get(0);
    }

    /**
     * add parms generation to a {@link WebHandler}
     */
    private WebHandler decorateWithParms(WebHandler delegate) {
        return new SessionParametersGenerator(delegate);
    }
}
