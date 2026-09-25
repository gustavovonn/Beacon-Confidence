#include <stdio.h>
#include <curl/curl.h>
#include <iostream>
 
int main(void)
{
  CURL *curl;
  CURLcode res;
 
  /* In windows, this will init the winsock stuff */
  curl_global_init(CURL_GLOBAL_ALL);
 
  /* get a curl handle */
  curl = curl_easy_init();
  if(curl) {
    /* First set the URL that is about to receive our POST. This URL can
       just as well be an https:// URL if that is what should receive the
       data. */
    
    /* Now specify the POST data */
    


    struct curl_slist *slist1 = NULL;
    slist1 = curl_slist_append(slist1, "Content-Type: application/json");
    slist1 = curl_slist_append(slist1, "Accept: application/json");
    
    /* set custom headers */
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, slist1);
    
    curl_easy_setopt(curl, CURLOPT_URL, "https://iot.ufsc.br/api/get.php");
    
    /* pass in a pointer to the data - libcurl does not copy */
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "{\"series\":{\"version\":\"1.2\",\"unit\":3298179364,\"r\":9999999,\"t0\":1672531200000000,\"tf\":1672531230000000,\"dev\":0,\"signature\":7777},\"credentials\":{\"domain\":\"tutorial\",\"username\":\"tutorial\",\"password\":\"tuto20182\"}}");
 
    /* Perform the request, res will get the return code */
    res = curl_easy_perform(curl);
    // std::cout<<"what";
    std::cout<<res<<"\n";
    /* Check for errors */
    if(res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));
 
    /* always cleanup */
    curl_easy_cleanup(curl);
  }
  curl_global_cleanup();
  return 0;
}