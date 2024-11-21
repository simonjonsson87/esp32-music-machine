df <- read.csv(col.names='x',"/Users/simonjonsson/ESP32-local/esp32-music-machine-v2/SimonsDebug/output.txt")
df$t <- seq(1,nrow(df))

plot(df$x ~ df$t)
