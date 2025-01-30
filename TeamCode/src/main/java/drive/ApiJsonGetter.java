package drive;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;

public class ApiJsonGetter {

    public String getJsonResponse(String apiUrl) throws Exception {
        HttpURLConnection connection = null;
        BufferedReader reader = null;

        try {
            // Create a URL object
            URL url = new URL(apiUrl);

            // Open a connection
            connection = (HttpURLConnection) url.openConnection();

            // Set the request method to GET
            connection.setRequestMethod("GET");

            // Set timeouts
            connection.setConnectTimeout(5000); // 5 seconds to establish connection
            connection.setReadTimeout(5000);    // 5 seconds to read data

            // Set headers (if necessary)
            connection.setRequestProperty("Accept", "application/json");

            // Check the response code
            int responseCode = connection.getResponseCode();
            if (responseCode != HttpURLConnection.HTTP_OK) {
                throw new IOException("HTTP GET Request Failed with Error Code: " + responseCode);
            }

            // Read the response using try-with-resources
            reader = new BufferedReader(new InputStreamReader(connection.getInputStream()));
            StringBuilder response = new StringBuilder();
            String line;

            while ((line = reader.readLine()) != null) {
                response.append(line);
            }

            return response.toString();

        } catch (IOException e) {
            // Log or handle the exception
            throw new IOException("Error occurred while connecting to API: " + e.getMessage(), e);

        } finally {
            // Ensure the connection and reader are closed
            if (reader != null) {
                try {
                    reader.close();
                } catch (IOException e) {
                    // Log or suppress closing exception
                }
            }
            if (connection != null) {
                connection.disconnect();
            }
        }
    }
}
