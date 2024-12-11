package org.firstinspires.ftc.teamcode.Utils;

import java.io.FileWriter;
import java.io.IOException;

public class FileSaver {
    public static void save(String fileName, String content) {
        try {
            FileWriter writer = new FileWriter(fileName);
            writer.write(content);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
