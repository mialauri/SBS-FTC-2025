package org.firstinspires.ftc.teamcode.Utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

public class FileReader {
    String fileName;

    public FileReader(String fileName) {
        this.fileName = fileName;
    }

    public List<String> getContents() {
        List<String> contents = new ArrayList<>();

        try {
            File file = new File(fileName);
            Scanner scanner = new Scanner(file);
            while (scanner.hasNextLine())
                contents.add(scanner.nextLine());

            scanner.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
        return contents;
    }
}
