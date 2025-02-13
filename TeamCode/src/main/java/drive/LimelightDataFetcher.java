package drive;

import java.lang.reflect.Type;
import java.util.HashMap;
import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.json.JSONArray;
import org.json.JSONObject;

public class LimelightDataFetcher {
    HashMap<Object, Object> map = new HashMap<>();
    Gson gson = new Gson();
    Type type = new TypeToken<HashMap<Object, Object>>() {}.getType();
    String jsonString;
    ApiJsonGetter jsonApi = new ApiJsonGetter();
    double[][] points;
    JSONArray ptsArray;
    public  JSONArray GetResults() throws Exception {
        // Replace with your Limelight's IP

        try {
            jsonString = jsonApi.getJsonResponse("http://172.29.0.1:5807/results");
        } catch (Exception e) {
            throw new Exception(e);
        }
        JSONObject jsonObject = new JSONObject(jsonString);
        JSONArray retroArray = jsonObject.getJSONArray("Retro");
        return retroArray;
    }
    public  double[][] getPoints() throws Exception {


        try {
            jsonString = jsonApi.getJsonResponse("http://172.29.0.1:5807/results");

            JSONObject jsonObject = new JSONObject(jsonString);
            JSONArray retroArray = jsonObject.getJSONArray("Retro");
            for (int i = 0; i < retroArray.length(); i++) {
                JSONObject retroObject = retroArray.getJSONObject(i);
                ptsArray = retroObject.getJSONArray("pts");
            }
            points = new double[ptsArray.length()][2];
            for (int j = 0; j < ptsArray.length(); j++) {
                JSONArray point = ptsArray.getJSONArray(j);
                double x = point.getDouble(0);
                double y = point.getDouble(1);
                points[j][0] = x;
                points[j][1] = y;
            }
            return points;
        } catch (Exception e) {
            return new double[0][];
        }
    }
}
