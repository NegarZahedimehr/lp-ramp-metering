import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

/**
 * Created by negar on 10/2/14.
 */
public class BatchWriter {
    private String path;
    private boolean append_to_file;
//    public WriteFile(String file_path){
//        path = file_path;
//    }

    public BatchWriter(String file_path, boolean append_value){
        path = file_path;
        append_to_file = append_value;
    }


    public void writeToFile(String textLine) throws IOException {
        FileWriter write = new FileWriter(path,append_to_file);
        PrintWriter prin_line = new PrintWriter(write);
        prin_line.print(textLine);
        prin_line.close();

    }

    public String getTableString ( int index, double[] dens_row, Double demand,boolean is_CTM, double TVH,double TVM){
        String table_row = (Integer.toString(index) + "\t");
        for (int i = 0; i < dens_row.length; i++) {
            table_row = table_row.concat(Double.toString(dens_row[i]) + "\t");
        }
        table_row = table_row + Double.toString(demand) + "\t" + (is_CTM ? Integer.toString(1) : Integer.toString(0)) + "\t" +
                Double.toString(TVH) + "\t" + Double.toString(TVM) + "\n";
        return table_row;
    }

}
