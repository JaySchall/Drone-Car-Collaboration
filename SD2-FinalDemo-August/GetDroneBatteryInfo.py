from selenium import webdriver
import time

# URL of the webpage
url = "http://192.168.11.1/clover/topics.html?topic=/mavros/battery"

# Initialize a WebDriver (make sure you have the appropriate WebDriver installed)
driver = webdriver.Chrome()  # You can replace 'Chrome' with the browser of your choice

# Navigate to the webpage
driver.get(url)

# Wait for the content to load (you might need to adjust the time)
time.sleep(5)

# Get the page source after waiting for it to load
page_source = driver.page_source

# Close the WebDriver when done
driver.quit()

# Now you can parse the page_source using BeautifulSoup or any other parsing method
# to extract the necessary data from the dynamically loaded content.
