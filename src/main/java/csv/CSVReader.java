package csv;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class CSVReader {

    private String path;

    public CSVReader(String path) {
        this.path = path;
    }

    public List<List<Double>> getAsDoubles() {
        List<List<Double>> doubleList = new ArrayList<>();
        for(List<String> list : getContents()) {
            doubleList.add(list.stream().map(s -> Double.valueOf(s)).collect(Collectors.toList()));
        }
        return doubleList;
    }

    public List<String> getLines() {
        List<String> lines = new ArrayList<>();
        InputStream fileInputStream = null;
        try {
            File csvFile = new File(path);
            fileInputStream = new FileInputStream(csvFile);
            BufferedReader fileReader = new BufferedReader(new InputStreamReader(fileInputStream));
            lines = fileReader.lines().skip(1).collect(Collectors.toList());
        } catch (Exception e) {
            e.printStackTrace();
        }
        return lines;
    }

    public List<List<String>> getContents() {
        return getLines().stream().map(l -> Arrays.asList(l.split(","))).collect(Collectors.toList());
    }


}
